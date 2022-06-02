#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdio.h>
#include "i2cll.h"

static QueueHandle_t i2c_queue;
static SemaphoreHandle_t i2c_mutex;

I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef i2cll_init() {
  static int i2c_initialized;
  if (!i2c_initialized) {
    i2c_initialized = 1;

    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x10909CEC;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_StatusTypeDef ret = HAL_I2C_Init(&hi2c2);
    if (ret != HAL_OK)
      return ret;
    /** Configure Analogue filter
     */
    ret = HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
    if (ret != HAL_OK)
      return ret;
    /** Configure Digital filter
     */
    ret = HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
    if (ret != HAL_OK)
      return ret;

    static StaticSemaphore_t i2c_mutex_buffer;
    i2c_mutex = xSemaphoreCreateMutexStatic(&i2c_mutex_buffer);
    static StaticQueue_t i2c_queue_buffer;
    static HAL_StatusTypeDef i2c_queue_storage_buffer[1];
    i2c_queue = xQueueCreateStatic(1, sizeof(HAL_StatusTypeDef),
                                   (uint8_t *)i2c_queue_storage_buffer,
                                   &i2c_queue_buffer);
  }

  return HAL_OK;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *dev) {
  (void)dev;
  long needs_resched = 0;
  HAL_StatusTypeDef status = HAL_OK;
  xQueueSendFromISR(i2c_queue, &status, &needs_resched);
  portYIELD_FROM_ISR(needs_resched);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *dev) {
  (void)dev;
  long needs_resched = 0;
  HAL_StatusTypeDef status = HAL_OK;
  xQueueSendFromISR(i2c_queue, &status, &needs_resched);
  portYIELD_FROM_ISR(needs_resched);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *dev) {
  (void)dev;
  long needs_resched = 0;
  HAL_StatusTypeDef status = HAL_ERROR;
  xQueueSendFromISR(i2c_queue, &status, &needs_resched);
  printf("ERROR CALLBACK\n");
  portYIELD_FROM_ISR(needs_resched);
}

HAL_StatusTypeDef i2cll_mem_read(uint16_t dev_addr, uint8_t addr, void *buffer,
                                 size_t len) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  HAL_StatusTypeDef ret =
      HAL_I2C_Mem_Read_IT(&hi2c2, dev_addr, addr, 1, buffer, len);
  if (ret == HAL_OK)
    xQueueReceive(i2c_queue, &ret, portMAX_DELAY);
  xSemaphoreGive(i2c_mutex);
  return ret;
}

HAL_StatusTypeDef i2cll_mem_write(uint16_t dev_addr, uint8_t addr, void *buffer,
                                  size_t len) {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  HAL_StatusTypeDef ret =
      HAL_I2C_Mem_Write_IT(&hi2c2, dev_addr, addr, 1, buffer, len);
  if (ret == HAL_OK)
    xQueueReceive(i2c_queue, &ret, portMAX_DELAY);
  xSemaphoreGive(i2c_mutex);
  return ret;
}