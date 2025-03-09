#include "ina219.h"
#include "stm32f7xx_hal.h"

// Write to INA219 Register
void INA219_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t value) {
    uint8_t buffer[2];
    buffer[0] = (value >> 8) & 0xFF;  // High Byte
    buffer[1] = value & 0xFF;         // Low Byte
    HAL_I2C_Mem_Write(hi2c, INA219_ADDRESS, reg, 1, buffer, 2, HAL_MAX_DELAY);
}

// Read from INA219 Register
uint16_t INA219_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t buffer[2];
    HAL_I2C_Mem_Read(hi2c, INA219_ADDRESS, reg, 1, buffer, 2, HAL_MAX_DELAY);
    return (buffer[0] << 8) | buffer[1];
}

// INA219 Initialization
void INA219_Init(I2C_HandleTypeDef *hi2c) {
    // Write calibration value to INA219
    INA219_WriteRegister(hi2c, INA219_REG_CALIBRATION, INA219_CALIBRATION_VALUE);
}

// Read Bus Voltage (in Volts)
float INA219_GetBusVoltage_mV(I2C_HandleTypeDef *hi2c) {
    uint16_t value = INA219_ReadRegister(hi2c, INA219_REG_BUSVOLTAGE);
    return ((float)(value >> 1)) * 0.001;  // Convert to Volts
}

// Read Shunt Voltage (in mV)
float INA219_GetShuntVoltage(I2C_HandleTypeDef *hi2c) {
    uint16_t value = INA219_ReadRegister(hi2c, INA219_REG_SHUNTVOLTAGE);
    return ((float)value) * 1.0;  // Convert to millivolts
}

// Read Current (in mA)
float INA219_GetCurrent_mA(I2C_HandleTypeDef *hi2c) {
    return (float)INA219_ReadRegister(hi2c, INA219_REG_CURRENT)*1.0;
}

// Read Power (in mW)
float INA219_GetPower(I2C_HandleTypeDef *hi2c) {
    return ((float)INA219_ReadRegister(hi2c, INA219_REG_POWER)) * 20;  // Convert to mW
}
