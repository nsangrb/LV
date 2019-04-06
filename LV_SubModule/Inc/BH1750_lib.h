#ifndef _BH1750_MFLIB_H_
#define _BH1750_MFLIB_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define Device_Address_L 0x47 // Device address when address pin LOW
#define Device_Address_H 0x5C // Device address when address pin LOW
//all command here taken from Data sheet OPECODE Table page 5
#define Power_Down 0x00

#define Power_On 0x01

#define Reset 0x07

#define Continuous_H_resolution_Mode  0x10

#define Continuous_H_resolution_Mode2  0x11

#define Continuous_L_resolution_Mode  0x13

#define OneTime_H_resolution_Mode  0x20

#define OneTime_H_resolution_Mode2  0x21

#define OneTime_L_resolution_Mode  0x23//As well as address value

void BH1750_Start(I2C_HandleTypeDef *hi2c);
void BH1750_Sleep(I2C_HandleTypeDef *hi2c);
void BH1750_SetMode(I2C_HandleTypeDef *hi2c,uint8_t mode);
void BH1750_Reset(I2C_HandleTypeDef *hi2c);
void BH1750_SetAddress(uint8_t add);
void BH1750_SetTime(uint32_t t);
uint16_t GetLightIntensity(I2C_HandleTypeDef *hi2c);

#endif 
