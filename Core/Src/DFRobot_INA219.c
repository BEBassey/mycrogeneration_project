#include "DFRobot_INA219.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;

/* ✅ Check if INA219 is responding */
uint8_t INA219_CheckDevice(uint8_t devAddr)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 3, HAL_MAX_DELAY) == HAL_OK);
}

/* ✅ Write to INA219 register */
void INA219_WriteRegister(uint8_t devAddr, uint8_t reg, uint16_t value)
{
    uint8_t data[2] = { (value >> 8) & 0xFF, value & 0xFF };
    HAL_I2C_Mem_Write(&hi2c1, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

/* ✅ Read from INA219 register */
uint16_t INA219_ReadRegister(uint8_t devAddr, uint8_t reg)
{
    uint8_t data[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return 0xFFFF;
    return (data[0] << 8) | data[1];
}

/* ✅ Initialize INA219 with updated config and calibration */
void INA219_Init(uint8_t devAddr)
{
    uint16_t config = 0x019F; // 32V, 320mV, 12-bit ADC, continuous
    INA219_WriteRegister(devAddr, INA219_REG_CONFIG, config);

    uint16_t calibration = 20480; // For 0.01Ω shunt and 0.2mA/bit
    INA219_WriteRegister(devAddr, INA219_REG_CALIBRATION, calibration);
}

/* ✅ Bus Voltage in V */
float INA219_GetBusVoltage(uint8_t devAddr)
{
    uint16_t raw = INA219_ReadRegister(devAddr, INA219_REG_BUSVOLTAGE);
    if (raw == 0xFFFF) return NAN;
    raw >>= 3;  // Remove OVF and CNVR bits
    return raw * 0.004f;
}

/* ✅ Shunt Voltage in mV */
float INA219_GetShuntVoltage(uint8_t devAddr)
{
    int16_t raw = INA219_ReadRegister(devAddr, INA219_REG_SHUNTVOLTAGE);
    if (raw == 0xFFFF) return NAN;
    return raw * 0.01f;
}

/* ✅ Current in mA (0.2mA per bit) */
float INA219_GetCurrent(uint8_t devAddr)
{
    int16_t raw = INA219_ReadRegister(devAddr, INA219_REG_CURRENT);
    if (raw == 0xFFFF) return NAN;
    return raw * 0.2f;
}

/* ✅ Power in mW (4mW per bit) */
float INA219_GetPower(uint8_t devAddr)
{
    int16_t raw = INA219_ReadRegister(devAddr, INA219_REG_POWER);
    if (raw == 0xFFFF) return NAN;
    return raw * 4.0f;
}

/* ✅ Set INA219 operating mode */
void INA219_SetMode(uint8_t devAddr, eInaMode_t mode)
{
    uint16_t conf = INA219_ReadRegister(devAddr, INA219_REG_CONFIG);
    if (conf == 0xFFFF) return;
    conf &= ~(0x07);
    conf |= mode;
    INA219_WriteRegister(devAddr, INA219_REG_CONFIG, conf);
}
