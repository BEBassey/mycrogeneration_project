#ifndef _BH1750_H
#define _BH1750_H

#include "stm32f7xx_hal.h"

#define BH1750_I2C_ADDR  (0x23 << 1)  // 8-bit STM32 format

/* BH1750 Commands */
#define BH1750_POWER_ON       0x01
#define BH1750_RESET          0x07
#define BH1750_CONT_HIGH_RES  0x10  // Continuous H-Resolution Mode

/* Public API */
uint8_t BH1750_Init(I2C_HandleTypeDef *hi2c);
uint16_t BH1750_ReadLux(I2C_HandleTypeDef *hi2c);

#endif /* _BH1750_H */
