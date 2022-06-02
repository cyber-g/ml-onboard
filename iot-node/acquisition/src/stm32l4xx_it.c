#include "stm32l4xx_it.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void DebugMon_Handler(void) {
}

void EXTI15_10_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void I2C2_EV_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&hi2c2);
}

void TIM7_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim7);
}

void QUADSPI_IRQHandler(void) {
  HAL_QSPI_IRQHandler(&hqspi);
}
