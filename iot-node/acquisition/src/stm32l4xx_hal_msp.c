#include "main.h"
#include "stm32l4xx_hal_qspi.h"
#include "stm32l4xx_hal_rcc.h"

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hi2c->Instance == I2C2) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = INTERNAL_I2C2_SCL_Pin | INTERNAL_I2C2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C2) {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(INTERNAL_I2C2_SCL_GPIO_Port, INTERNAL_I2C2_SCL_Pin);
    HAL_GPIO_DeInit(INTERNAL_I2C2_SDA_GPIO_Port, INTERNAL_I2C2_SDA_Pin);
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance == USART1) {
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = ST_LINK_UART1_TX_Pin | ST_LINK_UART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, ST_LINK_UART1_TX_Pin | ST_LINK_UART1_RX_Pin);
  }
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi) {
  (void) hqspi;
  __HAL_RCC_QSPI_CLK_ENABLE();
  HAL_NVIC_SetPriority(QUADSPI_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(QUADSPI_IRQn);
  __HAL_RCC_QSPI_FORCE_RESET();
  __HAL_RCC_QSPI_RELEASE_RESET();
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi) {
  (void) hqspi;
  HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
  __HAL_RCC_QSPI_CLK_DISABLE();
}
