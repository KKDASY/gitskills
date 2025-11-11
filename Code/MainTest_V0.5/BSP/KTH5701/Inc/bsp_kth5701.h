#ifndef __BSP_KTH5701_H__
#define __BSP_KTH5701_H__

#include "stm32g431xx.h"
#include "i2c.h"

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void KTH5701_BSP_Init(void);

#endif

