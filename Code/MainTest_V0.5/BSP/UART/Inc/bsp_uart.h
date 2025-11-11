#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "stm32g4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "stdbool.h"
#include "string.h"
#include "msuart.h"
#include "bsp_key.h"

#define RAW_FRAME_SIZE_MAX      1024
#define RAW_FRAME_HEADER        0xAA55
#define RAW_FRAME_TAIL          0xA55A

#define LOCAL_ID                0x00
#define SPINAL_ID               0xA0
#define DRIVER_BROADCAST_ID     0x0F
#define BOOT_JUMP_MAIN_CMD      0x01
#define BOOT_JUMP_SUB_CMD       0x00
#define BOOT_JUMP_PAYLOAD_LEN   0x0000

#define SDO_PROTOCOL            0x21
#define PREFIX_BYTE             0x00
#define SOURCE_PORT             0x00
#define DESTINATION_PORT        0x00
#define READ_CTRL_BYTE          0x70
#define CONNECT_BITMAP_INDEX    0x5100

#define CHECK_ONLINE_JOINT_ID   0x00

typedef struct
{
    volatile uint8_t rx_dma_buffer[RAW_FRAME_SIZE_MAX];
    volatile uint8_t tx_dma_buffer[RAW_FRAME_SIZE_MAX];
}UART_Buffers;

typedef enum
{
    Spinal_UART_Error_None = 0x00,
    Spinal_UART_Error_Parameter = 0x01,
    Spinal_UART_Error_Data = 0x02,
    Spinal_UART_Welcome_String = 0x03
}Spinal_UART_Error_e;

typedef enum {
  SDO_READ_WORD_OK_ID2 = 0x79,  ///< SDO read word ok response with 2 byte ID
} SDO_Control_Byte_e;

typedef enum
{
    ID_None = 0x00,
    ID_Main = 0x01,
    ID_Driver = 0x02,
    ID_Wire = 0x03,
    ID_Spinal = 0x04,
    ID_PC = 0x05,
    ID_LEN = 0x06
}ID_e;

//typedef enum
//{
//    HIGH_THUMB = 0x01,
//    HIGH_INDEX = 0x02,
//    HIGH_MIDDLE = 0x03,
//    HIGH_RING = 0x04,
//    HIGH_PINKY = 0x05,
//    HIGH_ALL = 0x0F
//}Boot_High_e;

//typedef enum
//{
//    LOW_JOINT_1 = 0x01,
//    LOW_JOINT_2 = 0x02,
//    LOW_JOINT_3 = 0x03,
//    LOW_JOINT_4 = 0x04,
//    LOW_ALL = 0x0F,
//    LOW_ALL_NO_SPINAL = 0x0E
//}Boot_Low_e;

void UART_RxCallback(MSUART *this);
bool Unpack(const uint8_t* rx_buffer, TestItem_t* test_item, ID_e *src_id, ID_e *dst_id);
uint16_t Pack(ID_e recv_id, uint8_t *tx_buffer, bool crc_check, TestItem_t test_item);
void BOOT_JUMP_Send(uint8_t id);
//void BOOT_Check_Online(void);
uint16_t SDO_Pack(uint8_t *tx_buffer, uint8_t dst_id, bool crc_check, uint16_t index, uint8_t sub_index);
Spinal_UART_Error_e SDO_Unpack(const uint8_t *rx_buffer);
bool Connection_Bitmap_Unpack(const uint32_t rx_buffer);
#endif
