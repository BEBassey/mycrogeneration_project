#ifndef __LCD_I2C_H__
#define __LCD_I2C_H__

#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>

/* Default I2C address for LCM1602 */
#define LCD_I2C_ADDR (0x27 << 1)  // If yours is different, change here

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);

#endif
