#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "DFRobot_INA219.h"
#include "BH1750.h"
#include "lcd_i2c.h"   // ✅ LCD support
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Function Declarations */
void SystemClock_Config(void);
void Scan_I2C_Devices(void);
void INA219_DebugFrame(uint8_t battery, uint8_t pv, uint8_t load, uint8_t light);
void INA219_Recalibrate(uint8_t devAddr);
void Error_Handler(void);

/* UART Buffer */
char uart_buf[512];

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();

    Scan_I2C_Devices();

    uint8_t battery_detected = INA219_CheckDevice(INA219_I2C_ADDRESS1);
    uint8_t pv_detected      = INA219_CheckDevice(INA219_I2C_ADDRESS2);
    uint8_t load_detected    = INA219_CheckDevice(INA219_I2C_ADDRESS3);

    if (battery_detected) {
        INA219_Init(INA219_I2C_ADDRESS1);
        INA219_Recalibrate(INA219_I2C_ADDRESS1);
    }
    if (pv_detected) {
        INA219_Init(INA219_I2C_ADDRESS2);
        INA219_Recalibrate(INA219_I2C_ADDRESS2);
    }
    if (load_detected) {
        INA219_Init(INA219_I2C_ADDRESS3);
        INA219_Recalibrate(INA219_I2C_ADDRESS3);
    }

    uint8_t light_sensor_ok = BH1750_Init(&hi2c1);

    // ✅ Initialize LCD
    LCD_Init(&hi2c1);
    LCD_Clear();
    LCD_Print("Power Monitor");
    HAL_Delay(1500);
    LCD_Clear();

    while (1)
    {
        INA219_DebugFrame(battery_detected, pv_detected, load_detected, light_sensor_ok);

        // ✅ LCD content
        float vbatt = INA219_GetBusVoltage(INA219_I2C_ADDRESS1);
        float vpv   = INA219_GetBusVoltage(INA219_I2C_ADDRESS2);
        float vload = INA219_GetBusVoltage(INA219_I2C_ADDRESS3);
        uint16_t lux = light_sensor_ok ? BH1750_ReadLux(&hi2c1) : 0;

        char lcd_line1[17];
        char lcd_line2[17];

        snprintf(lcd_line1, sizeof(lcd_line1), "Bat:%.2f PV:%.2f", vbatt, vpv);
        snprintf(lcd_line2, sizeof(lcd_line2), "Lux:%-4u Load:%s", lux, (vload > 0.1) ? "OK" : "--");

        LCD_SetCursor(0, 0); LCD_Print(lcd_line1);
        LCD_SetCursor(1, 0); LCD_Print(lcd_line2);

        HAL_Delay(1000);
    }
}

void Scan_I2C_Devices(void)
{
    for (uint8_t addr = 0x03; addr <= 0x77; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, HAL_MAX_DELAY) == HAL_OK)
        {
            sprintf(uart_buf, "I2C Device Found at 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        }
    }
}

void INA219_DebugFrame(uint8_t battery, uint8_t pv, uint8_t load, uint8_t light)
{
    char line[128];
    uart_buf[0] = '\0';

    strcat(uart_buf, "-------------------------------\r\n");
    strcat(uart_buf, "Sensor  | Volt  | Current | Power\r\n");
    strcat(uart_buf, "--------|-------|---------|--------\r\n");

    if (battery) {
        float v = INA219_GetBusVoltage(INA219_I2C_ADDRESS1);
        float i = INA219_GetCurrent(INA219_I2C_ADDRESS1);
        float p = INA219_GetPower(INA219_I2C_ADDRESS1);
        const char* dir = (i < 0) ? "↓" : "↑";
        sprintf(line, "Battery | %5.2fV | %6.1fmA%s | %5.2fW\r\n", v, fabsf(i), dir, fabsf(p));
        strcat(uart_buf, line);
    }

    if (pv) {
        float v = INA219_GetBusVoltage(INA219_I2C_ADDRESS2);
        float i = INA219_GetCurrent(INA219_I2C_ADDRESS2);
        float p = INA219_GetPower(INA219_I2C_ADDRESS2);
        const char* dir = (i < 0) ? "↓" : "↑";
        sprintf(line, "PV      | %5.2fV | %6.1fmA%s | %5.2fW\r\n", v, fabsf(i), dir, fabsf(p));
        strcat(uart_buf, line);
    }

    if (load) {
        float v = INA219_GetBusVoltage(INA219_I2C_ADDRESS3);
        float i = INA219_GetCurrent(INA219_I2C_ADDRESS3);
        float p = INA219_GetPower(INA219_I2C_ADDRESS3);
        const char* dir = (i < 0) ? "↓" : "↑";
        sprintf(line, "Load    | %5.2fV | %6.1fmA%s | %5.2fW\r\n", v, fabsf(i), dir, fabsf(p));
        strcat(uart_buf, line);
    }

    strcat(uart_buf, "-------------------------------\r\n");

    if (light) {
        uint16_t lux = BH1750_ReadLux(&hi2c1);
        sprintf(line, "Ambient Light: %u lux\r\n", lux);
        strcat(uart_buf, line);
    }

    strcat(uart_buf, "\r\n");

    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

void INA219_Recalibrate(uint8_t devAddr)
{
    INA219_WriteRegister(devAddr, INA219_REG_CALIBRATION, 20480);
    HAL_Delay(10);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
