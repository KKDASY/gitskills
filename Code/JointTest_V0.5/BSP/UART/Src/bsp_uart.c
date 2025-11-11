#include "bsp_uart.h"
#include "msuart.h"
#include "bsp_crc.h"
#include "bsp_tim.h"

extern Timeout_t timeout_manager;
extern TestItem_t result_feedback;

__attribute__((section(".RAM.AXI"))) UART_Buffers uart_buffers = {0};
TestItem_t test_item = {0};
bool test_item_valid = false;
uint32_t delta_time = 0;


void UART_RxCallback(MSUART *this)
{
    ID_e src_id = ID_None;
    MSUART_Start_DMA_Transfer(&msuart3, 0, 0, 0, 0, 0, 1, 1, 0, UART_RxCallback, (uint8_t*)uart_buffers.rx_dma_buffer, 12);
    if(Unpack((const uint8_t*)uart_buffers.rx_dma_buffer, &test_item, &src_id))
    {
        if(src_id == ID_Main)//来自主控设备
            test_item_valid = true;//标记测试项目有效
        else if(src_id == ID_Wire)//来自线缆设备
        {
            if(timeout_manager.waiting == true)//检查是否正在等待超时，如果超时，会Timeout_Handler中改为false
            {
								//收到响应后重置等待状态
                timeout_manager.waiting = false;
                timeout_manager.timeout = false;
								//设置RS485通信成功标志位
                result_feedback.bits.Item_RS485TxRx = 1;
            }
						//计算响应时间（当前时间戳-开始等待时间戳）
						delta_time = SystemTimer_GetTick_ms() - timeout_manager.tick_start;                                                                                 
        }
        return;
    }
    return;                                                                                                                                                                         
}

static uint16_t crc_calc = 0;
static uint16_t tail = 0;
bool Unpack(const uint8_t* rx_buffer, TestItem_t* test_item, ID_e *src_id)
{
    uint16_t header = (rx_buffer[0] << 8) | rx_buffer[1];
    if(header != RAW_FRAME_HEADER)
        return false;
    
    *src_id = rx_buffer[2];
    ID_e dst_id = rx_buffer[3];
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
    tx_buffer[2] = ID_Driver;
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

