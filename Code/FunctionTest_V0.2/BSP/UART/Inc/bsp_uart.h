#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "stm32g4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "stdbool.h"
#include "string.h"
#include "msuart.h"

#define RAW_FRAME_SIZE_MAX      1024
#define RAW_FRAME_HEADER        0xAA55
#define RAW_FRAME_TAIL          0xA55A

typedef struct
{
    volatile uint8_t rx_dma_buffer[RAW_FRAME_SIZE_MAX];
    volatile uint8_t tx_dma_buffer[RAW_FRAME_SIZE_MAX];
}UART_Buffers;

typedef enum
{
    ID_None = 0x00,
    ID_Main = 0x01,
    ID_Driver = 0x02,
    ID_Wire = 0x03,
    ID_Spinal = 0x04,
    ID_PC = 0x05,
    ID_LEN = 0x06,
    ID_BROADCAST = 0x0F,
}ID_e;

typedef union
{
    uint16_t raw_value;
    struct
    {
        uint8_t Item_BusVoltSample : 1;
        uint8_t Item_RS485TxRx : 1;
        uint8_t Item_InputEncoderSPI : 1;
        uint8_t Item_OutputEncoderI2C : 1;
        uint8_t Item_VrefSample : 1;
        uint8_t Item_PhaseCurrentSample : 1;
        uint8_t Item_DRV8316State : 1;
        uint8_t Item_DRV8316Drive : 1;
        uint8_t Item_FDCAN : 1;
        uint8_t Item_USB : 1;
        uint8_t Item_PHYVoltSample : 1;
        uint8_t Item_IMUSPI : 1;
        uint8_t Item_Reserved : 4;
    }bits;
}TestItem_t;

void UART_RxCallback(MSUART *this);
bool Unpack(const uint8_t* rx_buffer, TestItem_t* test_item, ID_e *src_id);
uint16_t Pack(ID_e recv_id, uint8_t *tx_buffer, bool crc_check, TestItem_t test_item, TestItem_t result_feedback);

#endif



