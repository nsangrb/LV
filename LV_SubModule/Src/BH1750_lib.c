#include "BH1750_lib.h"

static char address_value = 0x47;
bool state;
static uint32_t time=120;

void BH1750_Cmd_Write(I2C_HandleTypeDef *hi2c,uint8_t cmd)
{
	uint8_t data[1];
	data[0] = cmd;
	while(HAL_I2C_Master_Transmit(hi2c,address_value,data,1,time)!=HAL_OK);
	__HAL_I2C_CLEAR_FLAG(hi2c,I2C_FLAG_STOPF);
	HAL_Delay(100);
}
void BH1750_Start(I2C_HandleTypeDef *hi2c)
{
	BH1750_Cmd_Write(hi2c,Power_On);	
}  
void BH1750_Sleep(I2C_HandleTypeDef *hi2c)
{
	BH1750_Cmd_Write(hi2c,Power_Down);	
}
 void BH1750_Reset(I2C_HandleTypeDef *hi2c)
{
	BH1750_Cmd_Write(hi2c,Power_On);	
	BH1750_Cmd_Write(hi2c,Reset);	
}
void BH1750_SetAddress(uint8_t add)
{
	switch (add)
	{
      case Device_Address_L:
      address_value=Device_Address_L;
      state=false;
      break;
      case Device_Address_H:
      address_value=Device_Address_H;
      state=true;
      break;
    }	
}
void BH1750_SetTime(uint32_t t)
{
	time=t;
}
void BH1750_SetMode(I2C_HandleTypeDef *hi2c,uint8_t mode)
{
	 switch(mode){
        case Continuous_H_resolution_Mode:
        break;
        case Continuous_H_resolution_Mode2:
        break;
        case Continuous_L_resolution_Mode:       
        break;
        case OneTime_H_resolution_Mode:
        break;
        case OneTime_H_resolution_Mode2:
        break;
        case OneTime_L_resolution_Mode:  
        break;
    }
    HAL_Delay(10);
    BH1750_Cmd_Write(hi2c,mode);
}
uint16_t GetLightIntensity(I2C_HandleTypeDef *hi2c)
{   	
	uint8_t data_re[2] = {0,0};
	uint16_t physvalue;
	while(HAL_I2C_Master_Receive(hi2c,address_value,data_re,2,time*2)!=HAL_OK);
	HAL_Delay(100);
	physvalue =(data_re[0]<<8)+data_re[1];
	physvalue = physvalue/1.2;
	HAL_Delay(100);
	return physvalue;
}


