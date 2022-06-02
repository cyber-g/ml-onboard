#include "button.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include <FreeRTOS.h>
#include <semphr.h>

static SemaphoreHandle_t button_sem;
static volatile int latest_action;

void button_init() {
  static StaticSemaphore_t button_sem_buffer;
  button_sem = xSemaphoreCreateBinaryStatic(&button_sem_buffer);
}

int wait_for_button_press() {
  xSemaphoreTake(button_sem, portMAX_DELAY);
  return latest_action;
}

void stabilize_button(int ms) {
  while (xSemaphoreTake(button_sem, pdMS_TO_TICKS(ms)) == pdTRUE) {
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  (void)GPIO_Pin;
  static TickType_t pressed = 0;
  long needs_resched = 0;
  if (HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin)) {
    unsigned long delay = (xTaskGetTickCount() - pressed) / portTICK_PERIOD_MS;
    if (delay > 30) {
      latest_action = delay >= pdMS_TO_TICKS(200);
      xSemaphoreGiveFromISR(button_sem, &needs_resched);
    }
  } else
    pressed = xTaskGetTickCount();
  portYIELD_FROM_ISR(needs_resched);
}