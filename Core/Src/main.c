/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <ina219.h> // Include INA219 Library
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "bh1750_config.h"
#include "bmp2_config.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern BH1750_HandleTypeDef hbh1750_1;
extern struct bmp2_dev hbmp2_1;

// Use I2C_HandleTypeDef * instead of INA219_HandleTypeDef
I2C_HandleTypeDef *hpv_meter;
I2C_HandleTypeDef *hbatt_meter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read_Sensor_Data(void);
void Print_Sensor_Data(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        Read_Sensor_Data();
        Print_Sensor_Data();
    }
}

void Read_Sensor_Data(void)
{
    /* Read Light Intensity */
    BH1750_ReadIlluminance_lux(&hbh1750_1);

    /* Read Temperature from BMP280 */
    BMP2_ReadTemperature_degC(&hbmp2_1);
}

void Print_Sensor_Data(void)
{
    /* Read Sensors */
    float light_intensity = BH1750_ReadIlluminance_lux(&hbh1750_1);
    double temperature = BMP2_ReadTemperature_degC(&hbmp2_1);

    float pv_voltage_mV = INA219_GetBusVoltage_mV(&hi2c1);
    float pv_current_mA = INA219_GetCurrent_mA(&hi2c1);
    float pv_power_mW = pv_voltage_mV * pv_current_mA / 1000;

    float batt_voltage_mV = INA219_GetBusVoltage_mV(&hi2c1);
    float batt_current_mA = INA219_GetCurrent_mA(&hi2c1);
    float batt_power_mW = batt_voltage_mV * batt_current_mA / 1000;

    /* Debug Print */
    printf("DEBUG: Light: %.2f lux | Temp: %.2f C | PV: %.2f mV, %.2f mA, %.2f mW | Batt: %.2f mV, %.2f mA, %.2f mW\n",
           light_intensity, temperature,
           pv_voltage_mV, pv_current_mA, pv_power_mW,
           batt_voltage_mV, batt_current_mA, batt_power_mW);

    /* Format Output */
    char buffer[400];
    int n = sprintf(buffer,
        "\r\n"
        "==============================================\r\n"
        "  SENSOR DATA:\r\n"
        "==============================================\r\n"
        "  Light Intensity :   %.2f lux\r\n"
        "  --------------------------------------------\r\n"
        "  Temperature     :   %.2f °C\r\n"
        "  --------------------------------------------\r\n"
        "  PV Voltage      :   %.2f mV\r\n"
        "  PV Current      :   %.2f mA\r\n"
        "  PV Power        :   %.2f mW\r\n"
        "  --------------------------------------------\r\n"
        "  Battery Voltage :   %.2f mV\r\n"
        "  Battery Current :   %.2f mA\r\n"
        "  Battery Power   :   %.2f mW\r\n"
        "==============================================\r\n\r\n",
        light_intensity, temperature,
        pv_voltage_mV, pv_current_mA, pv_power_mW,
        batt_voltage_mV, batt_current_mA, batt_power_mW);

    /* Send to UART */
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, n, 100);
}


/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_TIM10_Init();

    /* USER CODE BEGIN 2 */
    BH1750_Init(&hbh1750_1);
    BMP2_Init(&hbmp2_1);

    // Assign INA219 sensors to I2C1
    hpv_meter = &hi2c1;
    hbatt_meter = &hi2c1;

    INA219_Init(hpv_meter);
    INA219_Init(hbatt_meter);

    HAL_TIM_Base_Start_IT(&htim10);
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the CPU, AHB, and APB bus clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB, and APB bus clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE END 4 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
