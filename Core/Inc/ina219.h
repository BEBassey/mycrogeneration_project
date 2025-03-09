#ifndef INA219_H
#define INA219_H

#include "stm32f7xx_hal.h"  // Change to match your STM32 series

// INA219 I2C Address (Default: 0x40)
#define INA219_ADDRESS 0x40 << 1  // Shifted for HAL

// INA219 Register Addresses
#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNTVOLTAGE  0x01
#define INA219_REG_BUSVOLTAGE    0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

// INA219 Calibration Value (for 32V, 2A range)
#define INA219_CALIBRATION_VALUE 4096

// Function Prototypes
void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_GetBusVoltage(I2C_HandleTypeDef *hi2c);
float INA219_GetShuntVoltage(I2C_HandleTypeDef *hi2c);
float INA219_GetCurrent(I2C_HandleTypeDef *hi2c);
float INA219_GetPower(I2C_HandleTypeDef *hi2c);

#endif
