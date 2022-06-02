#include <FreeRTOS.h>
#include <queue.h>
#include <string.h>
#include <task.h>

#include "font.h"
#include "main.h"
#include "matrix.h"

#define NB_MATRICES 3

static rgb_color matrices[64][NB_MATRICES];

const rgb_color RED = {.r = 0xff};
const rgb_color GREEN = {.g = 0xff};
const rgb_color BLUE = {.b = 0xff};
const rgb_color WHITE = {.r = 0xff, .g = 0xff, .b = 0xff};

QueueHandle_t empty_matrices;
QueueHandle_t full_matrices;

static inline void RST(int x) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, x); }

static inline void SB(int x) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, x); }

static inline void LAT(int x) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x); }

static inline void SCK(int x) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, x); }

static inline void SDA(int x) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x); }

static void ROW(int r, int x) {
  switch (r) {
  case 0:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, x);
    break;
  case 1:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, x);
    break;
  case 2:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, x);
    break;
  case 3:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, x);
    break;
  case 4:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, x);
    break;
  case 5:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, x);
    break;
  case 6:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, x);
    break;
  case 7:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, x);
    break;
  }
}

static void deactivate_rows(void) {
  for (int i = 0; i < 8; i++)
    ROW(i, 0);
}

static void pulse_SCK(void) {
  SCK(1);
  SCK(0);
}

static void pulse_LAT(void) {
  LAT(0);
  LAT(1);
}

static void activate_row(int r) { ROW(r, 1); }

static void send_byte(uint8_t val, int bank) {
  SB(bank);
  for (int i = 0; i < 8; i++) {
    SDA(val & 0x80);
    pulse_SCK();
    val <<= 1;
  }
}

static void mat_set_row(int r, const rgb_color val[8]) {
  int previous_row = (r + 7) % 8;
  for (int i = 0; i < 8; i++) {
    send_byte(val[i].b, 1);
    send_byte(val[i].g, 1);
    if (i == 7)
      ROW(previous_row, 0);
    send_byte(val[i].r, 1);
  }
  pulse_LAT();
  activate_row(r);
}

static void init_bank0(void) {
  for (int i = 0; i < 18; i++)
    send_byte(0xff, 0);
  pulse_LAT();
}

static void matrix_task() {
  for (int i = 1; i < NB_MATRICES; i++)
    dispose_matrix(matrices[i]);
  vTaskDelay(pdMS_TO_TICKS(100));
  RST(1);
  init_bank0();

  rgb_color *current_matrix = matrices[0];

  for (;;) {
    rgb_color *new_matrix;
    if (xQueueReceive(full_matrices, &new_matrix, pdMS_TO_TICKS(2)) == pdTRUE) {
      dispose_matrix(current_matrix);
      current_matrix = new_matrix;
    }
    for (int r = 0; r < 8; r++)
      mat_set_row(r, &current_matrix[r * 8]);
  }
}

void matrix_init(int prio) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {.Mode = GPIO_MODE_OUTPUT_PP,
                                      .Pull = GPIO_NOPULL,
                                      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                                      0};

  RST(0);
  LAT(1);
  SB(1);
  SCK(0);
  SDA(0);
  deactivate_rows();

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  static rgb_color *empty_matrices_storage_buffer[NB_MATRICES];
  static StaticQueue_t empty_matrices_buffer;
  empty_matrices = xQueueCreateStatic(NB_MATRICES, sizeof(rgb_color *),
                                      (uint8_t *)empty_matrices_storage_buffer,
                                      &empty_matrices_buffer);
  static rgb_color *full_matrices_storage_buffer[NB_MATRICES];
  static StaticQueue_t full_matrices_buffer;
  full_matrices = xQueueCreateStatic(NB_MATRICES, sizeof(rgb_color *),
                                     (uint8_t *)full_matrices_storage_buffer,
                                     &full_matrices_buffer);

#define STACK_SIZE 512
  static StackType_t stack_buffer[STACK_SIZE];
  static StaticTask_t task_buffer;
  xTaskCreateStatic(matrix_task, "matrix", STACK_SIZE, NULL, prio, stack_buffer,
                    &task_buffer);
}

rgb_color *get_empty_matrix(TickType_t delay) {
  rgb_color *matrix;
  if (xQueueReceive(empty_matrices, &matrix, delay) == pdTRUE) {
    memset(matrix, 0, 64 * sizeof(rgb_color));
    return matrix;
  }
  return NULL;
}

void display_matrix(rgb_color *matrix) {
  xQueueSendToBack(full_matrices, &matrix, 0);
}

void dispose_matrix(rgb_color *matrix) {
  xQueueSendToBack(empty_matrices, &matrix, 0);
}

void set_pixel(rgb_color *matrix, int r, int c, const rgb_color *p) {
  matrix[(7-r) * 8 + (7-c)] = *p;
}

void add_char(rgb_color *matrix, int r, int c, char s, const rgb_color *p) {
  const uint8_t *ch = char_for(s);
  if (!ch)
    return;
  for (int j = 0; j < 8; j++) {
    int rr = r + j;
    if (rr < 0 || rr >= 8)
      continue;
    for (int i = 0; i < 5; i++) {
      int cc = c + i;
      if (cc < 0 || cc >= 8)
        continue;
      if (ch[i] & (1 << (7 - j)))
        set_pixel(matrix, rr, cc, p);
    }
  }
}