#include <FreeRTOS.h>
#include <queue.h>
#include <stdint.h>
#include <task.h>

#include "main.h"
#include "morse.h"

static QueueHandle_t morse_queue;
static void morse_task(void *arg);

static const char *letters[] = {
    ".-",   "-...", "-.-.", "-..",  ".",   "..-.", "--.",  "....", "..",
    ".---", "-.-",  ".-..", "--",   "-.",  "---",  ".--.", "--.-", ".-.",
    "...",  "-",    "..-",  "...-", ".--", "-..-", "-.--", "--.."};

void morse_play(char c) { xQueueSendToBack(morse_queue, &c, portMAX_DELAY); }

void morse_init(int prio) {
#define QUEUE_SIZE 50
  static char morse_queue_storage_buffer[QUEUE_SIZE];
  static StaticQueue_t queue_buffer;
  morse_queue =
      xQueueCreateStatic(QUEUE_SIZE, sizeof(char),
                         (uint8_t *)morse_queue_storage_buffer, &queue_buffer);

#define STACK_SIZE 512
  static StackType_t morse_stack_buffer[STACK_SIZE];
  static StaticTask_t morse_task_buffer;
  xTaskCreateStatic(morse_task, "MORSE", STACK_SIZE, NULL, prio,
                    morse_stack_buffer, &morse_task_buffer);
}

static void morse_task(void *arg) {
  char c;
  // char *s;
  (void)arg;
start:
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
  xQueueReceive(morse_queue, &c, portMAX_DELAY);
decode:
  if (c < 'A' || c > 'Z')
    goto start;
  const char *s = letters[c - 'A'];
  for (;;) {
    for (const char *p = s; *p; p++) {
#define WAIT(n)                                                                \
  do {                                                                         \
    if (xQueueReceive(morse_queue, &c, pdMS_TO_TICKS(n)) == pdTRUE)            \
      goto decode;                                                             \
  } while (0)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
      WAIT(*p == '.' ? 200 : 500);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
      WAIT(100);
    }
    WAIT(1000);
  }
}