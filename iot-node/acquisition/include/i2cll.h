#ifndef I2CLL_H
#define I2CLL_H

#include "stm32l4xx_hal.h"

HAL_StatusTypeDef i2cll_init();
HAL_StatusTypeDef i2cll_mem_read(uint16_t dev_addr, uint8_t addr, void *buffer,
                                 size_t len);
HAL_StatusTypeDef i2cll_mem_write(uint16_t dev_addr, uint8_t addr, void *buffer,
                                  size_t len);

#endif // I2CLL_H