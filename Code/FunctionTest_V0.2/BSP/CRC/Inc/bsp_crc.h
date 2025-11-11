#ifndef __BSP_CRC_H__
#define __BSP_CRC_H__

#include "stm32g431xx.h"
#include "crc.h"
#include "stdbool.h"

#define SWAP_CRC_BYTE           false

void HW_CRC_Init(void);
uint16_t Get_CRC16_MODBUS(const uint8_t *data, uint16_t len);

#endif


