#include "test.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "bsp_led.h"
#include "bsp_tps26400.h"
#include "bsp_key.h"
#include "bsp_crc.h"
#include "bsp_uart.h"
#include "msuart.h"
#include "kth5701.h"
#include "bsp_kth5701.h"

extern TestedObj_e TestedObj_Current;
extern TestedObj_e TestedObj_Last;
extern bool BootJump_Process;
extern bool TPS26400_Async_Enable_Signal;
extern bool Enable_Process;
extern UART_Buffers uart_buffers;
extern RGBState_t led_state;
extern uint32_t tx_buzy;
extern uint32_t last_tx_buzy;

Timeout_t timeout_manager[ID_LEN] = {0};
TestItem_t test_item = {0};
bool EncoderI2C_Result = false;

void test(void)
{
  /* 初始化后执行 */
	//1.再次获取拨码开关状态，明确当前测试主体TestedObj_Current
	Read_Toggle_Switch();

	//2.配置测试项test_item
    TestItem_Config(TestedObj_Current, &test_item);
	
	//3.使能TPS26400电源芯片 触发条件：治具下拉手柄-->key4闭合
    TPS26400_Delay_Enable();
	
	//4.记录上一个测试对象（用于状态比较）
  //TestedObj_Last = TestedObj_Current;//似乎没有必要
	
	/* Bootloader跳转处理  KEY3 触发 */
   if(BootJump_Process)//需要跳转bootloader时
   {
			//根据测试对象选择目标设备
			if(TestedObj_Current == TestedObj_Driver)
					BOOT_JUMP_Send(DRIVER_BROADCAST_ID);//发送给驱动板
			else if(TestedObj_Current == TestedObj_Spinal)
					BOOT_JUMP_Send(SPINAL_ID);//发送给脊髓板
			BootJump_Process = false;
   }
		
		/* 主测试流程 */
    if(Enable_Process)//测试使能
    {
			//根据当前测试对象选择测试流程
        switch (TestedObj_Current) 
        {
            case TestedObj_Spinal://脊髓板测试
                SpinalTest_MainProcess(test_item);//未使用test_item
                YELLOW_ON();//黄色指示灯亮
                FLASH_ON;//闪光灯开启
                break;
            case TestedObj_Driver://驱动板测试
                DriverTest_MainProcess(test_item);
                YELLOW_ON();
                FLASH_ON;
                break;
            case TestedObj_MAWire://带磁编排线测试
								WireTest_MainProcess(test_item);
								YELLOW_ON();//黄色指示灯亮
                FLASH_ON;//闪光灯开启
                break;
            case TestedObj_NMAWire://不带磁编排线测试
                RS485TxRx_SubProcess();//RS485通信测试
								YELLOW_ON();//黄色指示灯亮
                FLASH_ON;//闪光灯开启
                break;
            case TestedObj_IMU:
                // IMUTest_MainProcess(test_item);
                break;
            default:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                break;
        }
				//检测发送状态变化：如果发送缓冲区有新数据待发送，tx_buzy从0->1时
//				if(tx_buzy > last_tx_buzy)
//				{
//					switch (TestedObj_Current) 
//					{
//						case TestedObj_Spinal:
//							SpinalTest_MainProcess(test_item);
//							YELLOW_ON();
//							FLASH_ON;
//							break;
//						case TestedObj_Driver:
//							DriverTest_MainProcess(test_item);
//							YELLOW_ON();
//							FLASH_ON;
//							break;
//						case TestedObj_MAWire:
//							RS485TxRx_SubProcess();//测试项改变？？？WireTest_MainProcess(test_item)
//							break;
//						case TestedObj_NMAWire:
//							RS485TxRx_SubProcess();
//							break;
//						case TestedObj_IMU:
//							// IMUTest_MainProcess(test_item);
//							break;
//						default:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
//							break;
//					}
//				}
        
        Enable_Process = false;
    }
    HAL_Delay(1);
}

void SpinalTest_MainProcess(TestItem_t test_item)
{
    //脊髓板超时管理（10s）
	timeout_manager[ID_Spinal].tick_start = SystemTimer_GetTick_ms();//起始时间
    timeout_manager[ID_Spinal].tick_end = timeout_manager[ID_Spinal].tick_start + TIMEOUT_MS;//超时时间
    timeout_manager[ID_Spinal].waiting = true;//标识开始等待
	//封装SDO数据包
	//函数说明：参数1-待发送数据  参数2-目标对象ID   参数3-是否CRC校验   参数4-索引号   参数5-子索引号
    uint16_t len = SDO_Pack((uint8_t *)uart_buffers.tx_dma_buffer, SPINAL_ID, true, CONNECT_BITMAP_INDEX, 0x00);
	//UART4发两次SDO协议数据包
    MSUART_Start_DMA_Transfer(
        &msuart4, true, true, NULL, 
        (const uint8_t *)uart_buffers.tx_dma_buffer, len, 
        true, true, false, UART_RxCallback, 
        (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
    HAL_Delay(100);
    MSUART_Start_DMA_Transfer(
        &msuart4, true, true, NULL, 
        (const uint8_t *)uart_buffers.tx_dma_buffer, len, 
        true, true, false, UART_RxCallback, 
        (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);

}

void DriverTest_MainProcess(TestItem_t test_item)
{
    timeout_manager[ID_Driver].tick_start = SystemTimer_GetTick_ms();//超时起始时间
    timeout_manager[ID_Driver].tick_end = timeout_manager[ID_Driver].tick_start + TIMEOUT_MS;//超时结束时间
    timeout_manager[ID_Driver].waiting = true;//标识开始等待
    uint16_t len = Pack(ID_Driver, (uint8_t *)uart_buffers.tx_dma_buffer, true, test_item);//封装
    MSUART_Start_DMA_Transfer(&msuart4, true, false, NULL, (const uint8_t *)uart_buffers.tx_dma_buffer, len, true, true, false, UART_RxCallback, (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);//发送
}

void WireTest_MainProcess(TestItem_t test_item)
{
  //I2C read
	EncoderI2C_Result = false;
  EncoderI2C_Result = OutputEncoderI2C_SubProcess();
  //RS485 Tx Rx
	SystemTimer_Delay_ms(10);
  RS485TxRx_SubProcess();
//	if(EncoderI2C_Result)
//	{
//		GREEN_ON();
//	}
//	else
//	{
//		RED_ON();
//	}
}

void IMUTest_MainProcess(TestItem_t test_item)
{

}
/**
* 多次读取KTH5701编码器的测量数据（TABZ.A）,统计有效数据占比（通过率），若通过率高于设定阈值（PASS_RATE），则认为编码器I2C输出正常
*/
bool OutputEncoderI2C_SubProcess(void)
{
    //1.触发编码器异步读取（预热/初始读取）
	KTH5701_Async_Read_Measurement(NULL);//异步读取编码器测量数据（不阻塞，立即返回）。作用是提前触发一次读取，“唤醒”编码器（避免首次读取因传感器未就绪导致数据无效）
		//2.初始化测试变量
    uint32_t ok = 0;//统计读取成功的次数
    float rate = 0;//有效数据率（通过率 = 有效次数/总次数）
	KTH5701_Raw_Measurement.TABZ.A = 0;//清零编码器数据缓冲区（避免上一次数据干扰）
	//3.循环多次读取编码器数据
    for(uint16_t i = 0;i < KTH5701_SAMPLE_TIMES;i++)
    {
        KTH5701_Async_Read_Measurement(NULL);//在HAL_I2C_MemRxCpltCallback回调函数中更新KTH5701_Raw_Measurement.TABZ.A
        if(KTH5701_Raw_Measurement.TABZ.A > 0 && KTH5701_Raw_Measurement.TABZ.A <= 0xFFFF)
            ok++;
				SystemTimer_Delay_ms(1);//延时1ms
    }
		//3.计算通过率并判断测试结果
    rate = (float)ok / (float)KTH5701_SAMPLE_TIMES;
    if(rate < PASS_RATE)
        return false;
    else
        return true;
}

void RS485TxRx_SubProcess(void)
{
    timeout_manager[ID_Wire].tick_start = SystemTimer_GetTick_ms();
    timeout_manager[ID_Wire].tick_end = timeout_manager[ID_Wire].tick_start + TIMEOUT_MS;
    timeout_manager[ID_Wire].waiting = true;
    timeout_manager[ID_Wire].timeout = false;
    TestItem_t item = {.raw_value = 0};
    uint16_t len = Pack(ID_Wire, (uint8_t *)uart_buffers.tx_dma_buffer, true, item);
    MSUART_Start_DMA_Transfer(&msuart4, true, false, NULL, 
        (const uint8_t *)uart_buffers.tx_dma_buffer, len, true, 
        true, false, UART_RxCallback, 
        (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
}

