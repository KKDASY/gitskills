#include "bsp_uart.h"
#include "msuart.h"
#include "bsp_crc.h"
#include "bsp_tim.h"
#include "bsp_led.h"
#include "bsp_tps26400.h"

extern Timeout_t timeout_manager[];
extern TestItem_t result_feedback;
extern CRC_HandleTypeDef hcrc;
extern TestedObj_e TestedObj_Current;
extern RGBState_t led_state;
extern bool EncoderI2C_Result;

__attribute__((section(".RAM.AXI"))) UART_Buffers uart_buffers = {0};
bool test_item_valid = false;

uint8_t sample;
void UART_RxCallback(MSUART *this)
{
	//1.重新开启串口接收 确保数据接收连续
	MSUART_Start_DMA_Transfer(&msuart4, 0, 0, 0, 0, 0, 1, 1, 0, UART_RxCallback, (uint8_t*)uart_buffers.rx_dma_buffer, 12);//重新开启，确保连续接收 每次12字节
	//2.检测帧头 是否为0xAA 0x55
	if(uart_buffers.rx_dma_buffer[0] == 170 && uart_buffers.rx_dma_buffer[1] == 85)
	{
		sample++;//统计有效帧数
	}
	//3.根据当前测试对象处理数据
    if(TestedObj_Current == TestedObj_Driver)//驱动板
    {
        TestItem_t ng_item;
        ID_e src_id = ID_None;
        ID_e dst_id = ID_None;
        bool result = Unpack((const uint8_t *)uart_buffers.rx_dma_buffer, &ng_item, &src_id, &dst_id);//从接收缓冲区解码ng_item  src_id  dst_id
        if(src_id == ID_Driver && dst_id == ID_Main)
        {
            timeout_manager[ID_Driver].waiting = false;//标记结束等待
            timeout_manager[ID_Driver].timeout = false;//标记未超时
            if(result)
                GREEN_ON();//ID匹配 常绿
            else
                RED_ON();//ID不匹配 常红
			TPS26400_Disable();
			
        }
    }
    else if(TestedObj_Current == TestedObj_Spinal)//脊髓板
    {
        Spinal_UART_Error_e result = SDO_Unpack((const uint8_t *)uart_buffers.rx_dma_buffer);//从接收缓冲区解码
        
        switch(result)
        {
            case Spinal_UART_Error_None://测试无误，常绿
				timeout_manager[ID_Spinal].waiting = false;
				timeout_manager[ID_Spinal].timeout = false;
                GREEN_ON();
                break;
            case Spinal_UART_Error_Parameter://测试参数错误，常黄
                YELLOW_ON();
				FLASH_ON;
                break;
            case Spinal_UART_Error_Data://测试数据错误，常红
				timeout_manager[ID_Spinal].waiting = false;
				timeout_manager[ID_Spinal].timeout = false;
                RED_ON();
                break;
            case Spinal_UART_Welcome_String:
                break;
        }
    }
    else if(TestedObj_Current == TestedObj_MAWire || TestedObj_Current == TestedObj_NMAWire)//主线缆  非主线缆
    {
        ID_e src_id = ID_None;
        ID_e dst_id = ID_None;
        TestItem_t item = {.raw_value = 0};
        bool result = Unpack((const uint8_t *)uart_buffers.rx_dma_buffer, &item, &src_id, &dst_id);//从接收缓冲区解码ng_item  src_id  dst_id
				if(result)
				{
					if(src_id == ID_Wire && dst_id == ID_Main && timeout_manager[ID_Wire].waiting)
					{
							timeout_manager[ID_Wire].waiting = false;//标记结束等待
							timeout_manager[ID_Wire].timeout = false;//标记未超时
							if(TestedObj_Current == TestedObj_MAWire)//如果是主线缆  还需要进一步操作
							{
									if(EncoderI2C_Result)//编码器I2C校验通过
										GREEN_ON();//常绿
									else
										RED_ON();//常红
							}
							else//如果不是主线缆
							{
										GREEN_ON();//常绿
							}
					}
				}
				else
				{
						RED_ON();
				}
    }
}

bool Unpack(const uint8_t* rx_buffer, TestItem_t* test_item, ID_e *src_id, ID_e *dst_id)
{
    uint16_t header = (rx_buffer[0] << 8) | rx_buffer[1];
    if(header != RAW_FRAME_HEADER)
        return false;
    
    *src_id = (ID_e)rx_buffer[2];
    *dst_id = (ID_e)rx_buffer[3];
    if(*dst_id != ID_Main)
        return false;

	uint16_t tail = (rx_buffer[8] << 8) | rx_buffer[9];
    if(tail != RAW_FRAME_TAIL)
        return false;

	uint16_t crc = (rx_buffer[10] << 8) | rx_buffer[11];
    uint16_t crc_calc = Get_CRC16_MODBUS(rx_buffer, 10);
    if (crc != crc_calc) return false;

    if(*src_id == ID_Driver || *src_id == ID_Wire)
    {
        uint16_t command = (rx_buffer[4] << 8) | rx_buffer[5];
        TestItem_t temp;
        temp.raw_value = command & 0x00FF;//待测试项目

        uint16_t feedback = (rx_buffer[6] << 8) | rx_buffer[7];
        test_item->raw_value = feedback & 0x00FF;//测试项目

        timeout_manager[ID_Driver].waiting = false;
        timeout_manager[ID_Driver].timeout = false;

        if (temp.raw_value == test_item->raw_value) 
            return true;
    }
    return false;
} 

static void Pack_TestItem(TestItem_t test_item, uint8_t *command_buffer, uint8_t len) 
{
    if (len != 2) return;
    command_buffer[0] = ((uint8_t *)&test_item)[1];
    command_buffer[1] = ((uint8_t *)&test_item)[0]; 
}

uint16_t Pack(ID_e recv_id, uint8_t *tx_buffer, bool crc_check, TestItem_t test_item) 
{
    uint16_t len = 0;
    tx_buffer[0] = (RAW_FRAME_HEADER >> 8) & 0xFF;
    tx_buffer[1] = (RAW_FRAME_HEADER) & 0xFF;
    tx_buffer[2] = ID_Main;
    tx_buffer[3] = recv_id;
    Pack_TestItem(test_item, tx_buffer + 4, 2);
    tx_buffer[6] = 0x00;
    tx_buffer[7] = 0x00;
    tx_buffer[8] = (RAW_FRAME_TAIL >> 8) & 0xFF;
    tx_buffer[9] = (RAW_FRAME_TAIL) & 0xFF;
    len += 10;
    if (crc_check) 
    {
        uint16_t crc = Get_CRC16_MODBUS(tx_buffer, 10);
        tx_buffer[10] = (crc >> 8) & 0xFF;
        tx_buffer[11] = (crc) & 0xFF;

        len += 2;
    } 

    return len;
}


void BOOT_JUMP_Send(uint8_t id)
{
    uart_buffers.tx_dma_buffer[0] = (RAW_FRAME_HEADER >> 8) & 0xFF;//帧头
    uart_buffers.tx_dma_buffer[1] = (RAW_FRAME_HEADER) & 0xFF;
    uart_buffers.tx_dma_buffer[2] = LOCAL_ID;//本地设备ID
    uart_buffers.tx_dma_buffer[3] = id;//目标设备ID
    uart_buffers.tx_dma_buffer[4] = BOOT_JUMP_MAIN_CMD;//主命令
    uart_buffers.tx_dma_buffer[5] = BOOT_JUMP_SUB_CMD;//子命令
    uart_buffers.tx_dma_buffer[6] = (BOOT_JUMP_PAYLOAD_LEN >> 8) & 0xFF;//负载长度
    uart_buffers.tx_dma_buffer[7] = (BOOT_JUMP_PAYLOAD_LEN) & 0xFF;
    uint16_t crc = cal_crc16_ccitt((const uint8_t*)uart_buffers.tx_dma_buffer, 8);//CRC校验
    uart_buffers.tx_dma_buffer[8] = (crc >> 8) & 0xFF;//校验值
    uart_buffers.tx_dma_buffer[9] = (crc) & 0xFF;
    MSUART_Start_DMA_Transfer(&msuart4, true, false, NULL, (const uint8_t*)uart_buffers.tx_dma_buffer, 10, true, true, false, UART_RxCallback, (uint8_t*)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
}

uint16_t SDO_Pack(uint8_t *tx_buffer, uint8_t dst_id, bool crc_check, uint16_t index, uint8_t sub_index)
{
    tx_buffer[0] = (RAW_FRAME_HEADER >> 8) & 0xFF;
    tx_buffer[1] = (RAW_FRAME_HEADER) & 0xFF;
    tx_buffer[2] = LOCAL_ID;
    tx_buffer[3] = dst_id;
    uint8_t len = 0;
    uint16_t max_recv_window = 0xFF;
    uint16_t control_word = ((len & 0x3F) << 10) | (max_recv_window & 0x3FF);
    tx_buffer[4] = (control_word >> 8) & 0xFF;
    tx_buffer[5] = (control_word) & 0xFF;
    tx_buffer[6] = SDO_PROTOCOL;
    tx_buffer[7] = PREFIX_BYTE;
    tx_buffer[8] = READ_CTRL_BYTE;
    tx_buffer[9] = (index >> 8) & 0xFF;
    tx_buffer[10] = (index) & 0xFF;
    tx_buffer[11] = sub_index;
    tx_buffer[12] = 0x00;
    tx_buffer[13] = 0x00;
    uint16_t crc = Get_CRC16_MODBUS((const uint8_t*)tx_buffer, 14);
    tx_buffer[14] = (crc >> 8) & 0xFF;
    tx_buffer[15] = (crc) & 0xFF;

    return (len + 1) * 16;
}

Spinal_UART_Error_e SDO_Unpack(const uint8_t* rx_buffer)
{
    uint16_t header = (rx_buffer[0] << 8) | rx_buffer[1];
    if(header != RAW_FRAME_HEADER)//判断帧头数据是否为0xAA55
        return Spinal_UART_Welcome_String;

    uint8_t src_id = rx_buffer[2];
    uint8_t dst_id = rx_buffer[3];
    if(dst_id != LOCAL_ID || src_id != SPINAL_ID)
        return Spinal_UART_Error_Parameter;//ID错误，返回参数错误Spinal_UART_Error_Parameter

    uint16_t control_word = (rx_buffer[4] << 8) | rx_buffer[5];
    uint8_t len_byte = (control_word >> 10) & 0x3F;
    uint16_t len = (len_byte + 1) * 16;
    uint16_t max_recv_window = control_word & 0x3FF;

    uint16_t crc = (rx_buffer[len - 2] << 8) | rx_buffer[len - 1];
    uint16_t crc_calc = Get_CRC16_MODBUS(rx_buffer, len - 2);
    if(crc != crc_calc)
        return Spinal_UART_Error_Parameter;//CRC校验错误，返回参数错误Spinal_UART_Error_Parameter

    uint8_t protocol = rx_buffer[6];
    if(protocol != SDO_PROTOCOL)
        return Spinal_UART_Error_Parameter;//协议错误，返回参数错误Spinal_UART_Error_Parameter

    uint8_t prefix_byte = rx_buffer[7];
    uint8_t control_byte = rx_buffer[8];

    uint16_t index = (rx_buffer[9] << 8) | rx_buffer[10];
    uint8_t sub_index = rx_buffer[11];
    switch(control_byte)
    {
        case SDO_READ_WORD_OK_ID2:
        {
						uint32_t id = (rx_buffer[12] << 8) | rx_buffer[13];
            uint32_t data = (rx_buffer[14]) | (rx_buffer[15] << 8) | (rx_buffer[16] << 16) | (rx_buffer[17] << 24);
            if(index == CONNECT_BITMAP_INDEX && sub_index == 0x00)
            {
                if(Connection_Bitmap_Unpack(data))
                    return Spinal_UART_Error_None;//无错误，返回Spinal_UART_Error_None
                else
                    return Spinal_UART_Error_Data;//数据错误，返回Spinal_UART_Error_Data
            }
            break;  
				}      
        default:
            return Spinal_UART_Error_Data;//control_byte不对，返回数据错误Spinal_UART_Error_Data
    }
    return Spinal_UART_Error_Data;//index  sub_index错误，返回数据错误Spinal_UART_Error_Data
}

bool Connection_Bitmap_Unpack(const uint32_t rx_buffer)
{
    uint32_t bitmap = rx_buffer;
    for(uint8_t i = 0; i < 5; i++)
    {
        if(bitmap & (1 << (i * 4 + CHECK_ONLINE_JOINT_ID)))
            continue;
        else
            return false;
        // for(uint8_t j = 0;j < 4; j++)
        // {
        //     if(bitmap & (1 << (i * 4 + j)))
        //         continue;
        //     else
        //         return false;
        // }
    }
}


