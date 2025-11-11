#include "bsp_uart.h"
#include "msuart.h"
#include "bsp_crc.h"
#include "bsp_tim.h"
#include <stdbool.h>

extern Timeout_t timeout_manager;
extern TestItem_t result_feedback;

__attribute__((section(".RAM.AXI"))) UART_Buffers uart_buffers = {0};
TestItem_t test_item = {0};
bool test_item_valid = false;

void UART_RxCallback(MSUART *this)
{
	// 重新启动 DMA 接收（保证一直能收）
	MSUART_Start_DMA_Transfer(&msuart3, 0, 0, 0, 0, 0,
																			1, 1, 0, UART_RxCallback, 
																			(uint8_t*)uart_buffers.rx_dma_buffer, 12);
	
	// 从接收到的帧中提取信息
	ID_e dst_id = (ID_e)uart_buffers.rx_dma_buffer[3];
  ID_e src_id = (ID_e)uart_buffers.rx_dma_buffer[2];
	
	// 判断帧目的地址是否发给我或广播 ID_Wire = 0x03  ID_BROADCAST = 0x0F
	if(dst_id != ID_Wire && dst_id != ID_BROADCAST)
		return;
//    if(src_id != ID_Driver)
//        return;
	
	 
	// 构造回复帧
  TestItem_t temp = {.raw_value = 0};
  uint16_t len = Pack(src_id, (uint8_t*)uart_buffers.tx_dma_buffer, true, temp, temp);
	
	// 用DMA发送该帧，并再次打开接收
  MSUART_Start_DMA_Transfer(&msuart3, true, true, NULL, (const uint8_t*)uart_buffers.tx_dma_buffer, len,
																			true, true, false, UART_RxCallback, (uint8_t*)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
    return;
}

static uint16_t crc_calc = 0;
static uint16_t tail = 0;
bool Unpack(const uint8_t* rx_buffer, TestItem_t* test_item, ID_e *src_id)
{
    uint16_t header = (rx_buffer[0] << 8) | rx_buffer[1];
    if(header != RAW_FRAME_HEADER)
        return false;
    
    *src_id = (ID_e)rx_buffer[2];
    ID_e dst_id = (ID_e)rx_buffer[3];
    if(dst_id != ID_Driver)
        return false;

	tail = (rx_buffer[8] << 8) | rx_buffer[9];
    if(tail != RAW_FRAME_TAIL)
        return false;

	uint16_t crc = (rx_buffer[10] << 8) | rx_buffer[11];
    crc_calc = Get_CRC16_MODBUS(rx_buffer, 10);
    if(crc != crc_calc)
        return false;

    if(*src_id == ID_Main)
    {
        uint16_t command = (rx_buffer[4] << 8) | rx_buffer[5];
        test_item->raw_value = command & 0x00FF;
    }
    return true;
} 

static void Pack_TestItem(TestItem_t test_item, uint8_t *command_buffer, uint8_t len) 
{
    if (len != 2) return;
    command_buffer[0] = ((uint8_t *)&test_item)[1];
    command_buffer[1] = ((uint8_t *)&test_item)[0]; 
}

uint16_t Pack(ID_e recv_id, uint8_t *tx_buffer, bool crc_check, TestItem_t test_item, TestItem_t result_feedback) 
{
    uint16_t len = 0;
    tx_buffer[0] = (RAW_FRAME_HEADER >> 8) & 0xFF;
    tx_buffer[1] = (RAW_FRAME_HEADER) & 0xFF;
    tx_buffer[2] = ID_Wire;
    tx_buffer[3] = recv_id;
    Pack_TestItem(test_item, tx_buffer + 4, 2);
    Pack_TestItem(result_feedback, tx_buffer + 6, 2);
    if (crc_check) 
    {
        tx_buffer[8] = (RAW_FRAME_TAIL >> 8) & 0xFF;
        tx_buffer[9] = (RAW_FRAME_TAIL) & 0xFF;

        uint16_t crc = Get_CRC16_MODBUS(tx_buffer, 10);
        tx_buffer[10] = (crc >> 8) & 0xFF;
        tx_buffer[11] = (crc) & 0xFF;

        len += 12;
    } 
    else 
    {
        tx_buffer[8] = (RAW_FRAME_TAIL >> 8) & 0xFF;
        tx_buffer[9] = (RAW_FRAME_TAIL) & 0xFF;
        len += 10;
    }

    return len;
}


