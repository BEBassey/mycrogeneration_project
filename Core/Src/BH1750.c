#include "BH1750.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"  // For UART debug output

uint8_t BH1750_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd;

    // Power on
    cmd = BH1750_POWER_ON;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return 0;

    HAL_Delay(10);  // Delay between commands

    // Reset
    cmd = BH1750_RESET;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return 0;

    HAL_Delay(10);  // Give reset time to settle

    // Continuous High Resolution Mode (1 lx resolution)
    cmd = BH1750_CONT_HIGH_RES;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return 0;

    HAL_Delay(200);  // Allow time for first measurement to complete

    return 1;  // Initialization successful
}

uint16_t BH1750_ReadLux(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {0};
    if (HAL_I2C_Master_Receive(hi2c, BH1750_I2C_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return 0xFFFF;

    // Optional debug output
    char buf[64];
    sprintf(buf, "BH1750 Raw: %02X %02X\r\n", data[0], data[1]);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    return ((uint16_t)data[0] << 8) | data[1];  // Combine MSB + LSB
}
