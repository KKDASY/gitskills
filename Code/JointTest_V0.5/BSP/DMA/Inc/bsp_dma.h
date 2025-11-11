#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32g4xx_hal.h"
#include "main.h"
#include "stdbool.h"

uint32_t DMA_Get_Request_Selection_SPI(SPI_TypeDef* spi, bool is_tx);
void DMA_Set_Config(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
uint32_t DMA_Get_Request_Selection_I2C(I2C_TypeDef *i2c, bool is_tx);

#endif
