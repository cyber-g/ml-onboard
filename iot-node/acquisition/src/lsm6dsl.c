#include "lsm6dsl.h"
#include "activity.h"
#include "i2cll.h"
#include <math.h>
#include <queue.h>
#include <task.h>

static QueueHandle_t lsm6dsl_queue;

#define IMU 0xd4

static void lsm6dsl_task(void *arg) {
  int freq_ms = (int)arg;

  // Ensure auto-increment is on so that we can the configuration at once
  uint8_t b = 0x04;
  HAL_StatusTypeDef ret = i2cll_mem_write(IMU, 0x12, &b, 1);
  configASSERT(ret == HAL_OK);

  uint8_t ctrl[] = {
      0x88, // CTRL1_XL: 1.66kHz, 4g scale
      0x00, // CTRL2_G
      0x44, // CTRL3_C:  BDU mode + auto-increment
      0x00, // CTRL4_C
      0x00, // CTRL5_C
      0x00, // CTRL6_C:  high performance XL mode
      0x00, // CTRL7_G
      0x00, // CTRL8_XL
      0x00, // CTRL9_XL
      0x00, // CTRL10_C
  };
  b = 0x04;
  ret = i2cll_mem_write(IMU, 0x12, &b, 1);
  configASSERT(ret == HAL_OK);
  ret = i2cll_mem_write(IMU, 0x10, ctrl, sizeof ctrl);
  configASSERT(ret == HAL_OK);
  b = 0x03;
  ret = i2cll_mem_write(IMU, 0x1a, &b, 1);

  TickType_t wake_time = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(freq_ms));
    xl_data data;
    ret = i2cll_mem_read(IMU, 0x28, (uint8_t *)&data.xl, sizeof data.xl);
    data.timestamp = xTaskGetTickCount();
    data.activity = current_activity;
    configASSERT(ret == HAL_OK);
    xQueueSendToBack(lsm6dsl_queue, &data, 0);
  }
}

void lsm6dsl_read_accel(xl_data *data) {
  xQueueReceive(lsm6dsl_queue, data, portMAX_DELAY);
}

void lsm6dsl_init(int prio, int freq_ms) {
  i2cll_init();

#define QUEUE_SIZE 50
  static xl_data lsm6dsl_queue_storage_buffer[QUEUE_SIZE];
  static StaticQueue_t queue_buffer;
  lsm6dsl_queue = xQueueCreateStatic(QUEUE_SIZE, sizeof(xl_data),
                                     (uint8_t *)lsm6dsl_queue_storage_buffer,
                                     &queue_buffer);

#define STACK_SIZE 512
  static StackType_t lsm6dsl_stack_buffer[STACK_SIZE];
  static StaticTask_t lsm6dsl_task_buffer;
  xTaskCreateStatic(lsm6dsl_task, "IMU", STACK_SIZE, (void *)freq_ms, prio,
                    lsm6dsl_stack_buffer, &lsm6dsl_task_buffer);
}
