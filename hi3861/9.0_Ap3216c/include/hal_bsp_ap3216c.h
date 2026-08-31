#ifndef __HAL_BSP_AP3216C_H__
#define __HAL_BSP_AP3216C_H__

#include "cmsis_os2.h"

#define AP3216C_I2C_ADDR (0x3C)   // 8位写地址 (7位 0x1E)
#define AP3216C_I2C_IDX  WIFI_IOT_I2C_IDX_0
#define AP3216C_I2C_SPEED 400000

uint32_t AP3216C_Init(void);
uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData);
uint32_t AP3216C_WriteReg(uint8_t regAddr, uint8_t byte);
uint32_t AP3216C_ReadReg(uint8_t regAddr, uint8_t *byte);

#endif
