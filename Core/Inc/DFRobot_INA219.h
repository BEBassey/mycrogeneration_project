#ifndef _DFROBOT_INA219_H
#define _DFROBOT_INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include "math.h"  // ✅ For NAN

/* ✅ I2C Addresses (8-bit shifted for STM32 HAL) */
#define INA219_I2C_ADDRESS1  (0x40 << 1)  // Battery
#define INA219_I2C_ADDRESS2  (0x45 << 1)  // PV
#define INA219_I2C_ADDRESS3  (0x41 << 1)  // ✅ Load (updated)

/* Register Map */
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNTVOLTAGE   0x01
#define INA219_REG_BUSVOLTAGE     0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05

/* Operating Modes */
typedef enum {
    eIna219PowerDown,
    eIna219SVolTrig,
    eIna219BVolTrig,
    eIna219SAndBVolTrig,
    eIna219AdcOff,
    eIna219SVolCon,
    eIna219BVolCon,
    eIna219SAndBVolCon
} eInaMode_t;

/* API Functions */
void INA219_Init(uint8_t devAddr);
uint8_t INA219_CheckDevice(uint8_t devAddr);
float INA219_GetBusVoltage(uint8_t devAddr);
float INA219_GetShuntVoltage(uint8_t devAddr);
float INA219_GetCurrent(uint8_t devAddr);
float INA219_GetPower(uint8_t devAddr);
void INA219_SetMode(uint8_t devAddr, eInaMode_t mode);

/* Low-Level Register Access */
uint16_t INA219_ReadRegister(uint8_t devAddr, uint8_t reg);
void INA219_WriteRegister(uint8_t devAddr, uint8_t reg, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* _DFROBOT_INA219_H */
