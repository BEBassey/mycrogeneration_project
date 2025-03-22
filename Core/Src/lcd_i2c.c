#include "lcd_i2c.h"

static I2C_HandleTypeDef *_lcd_hi2c;
static uint8_t lcd_backlight = 0x08;

#define LCD_COMMAND 0
#define LCD_DATA    1
#define ENABLE      0x04

static void LCD_WriteNibble(uint8_t nibble, uint8_t mode);
static void LCD_SendByte(uint8_t byte, uint8_t mode);
static void LCD_SendCmd(uint8_t cmd);

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    _lcd_hi2c = hi2c;
    HAL_Delay(50);

    LCD_WriteNibble(0x30, LCD_COMMAND); HAL_Delay(5);
    LCD_WriteNibble(0x30, LCD_COMMAND); HAL_Delay(5);
    LCD_WriteNibble(0x30, LCD_COMMAND); HAL_Delay(5);
    LCD_WriteNibble(0x20, LCD_COMMAND); HAL_Delay(5); // 4-bit

    LCD_SendCmd(0x28); // 2-line, 5x8 dots
    LCD_SendCmd(0x0C); // Display on, cursor off
    LCD_SendCmd(0x06); // Entry mode
    LCD_Clear();
}

void LCD_Clear(void)
{
    LCD_SendCmd(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    const uint8_t row_offsets[] = {0x00, 0x40};
    LCD_SendCmd(0x80 | (col + row_offsets[row]));
}

void LCD_Print(char *str)
{
    while (*str)
        LCD_SendByte(*str++, LCD_DATA);
}

static void LCD_SendCmd(uint8_t cmd)
{
    LCD_SendByte(cmd, LCD_COMMAND);
}

static void LCD_SendByte(uint8_t byte, uint8_t mode)
{
    LCD_WriteNibble(byte & 0xF0, mode);
    LCD_WriteNibble((byte << 4) & 0xF0, mode);
}

static void LCD_WriteNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | lcd_backlight | mode;
    uint8_t data_enable = data | ENABLE;
    uint8_t data_disable = data & ~ENABLE;

    HAL_I2C_Master_Transmit(_lcd_hi2c, LCD_I2C_ADDR, &data_enable, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(_lcd_hi2c, LCD_I2C_ADDR, &data_disable, 1, HAL_MAX_DELAY);
}
