#include "bsp_kth5701.h"
#include "kth5701.h"
#include "bsp_led.h"
extern RGBState_t led_state;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Mem_Rx_Cplt_Callback_Process();//调用update_measurement()根据rx_buffer更新KTH5701_Raw_Measurement
	
	KTH5701_Error_Counter = 0;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Master_Tx_Cplt_Callback_Process();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Master_Rx_Cplt_Callback_Process();
	
	KTH5701_Error_Counter = 0;
}
/**
* 两步配置（寄存器配置、工作模式切换），确保传感器正常工作，否则红灯（常亮）
* 1.寄存器初始化配置
* 2.切换模式到连续接收模式
*/
void KTH5701_BSP_Init(void)
{
    uint8_t kth5701_ok = 0;//状态标记：记录初始化成功的步骤数
    KTH5701_Init();//配置I2C DMA发送和接收
	//1.设置寄存器
    for(int i = 0;i < 100;i++)
    {
        if(KTH5701_Config_Registers())
        {
            kth5701_ok += 1;
            break;
        }
        HAL_Delay(5);
    }
    if(kth5701_ok != 1)
    {
		//Error_Handler();
        RED_ON();
    }	
		//2.切换模式到连续接收模式
    for(int i = 0;i < 100;i++)
    {
        if(KTH5701_Block_Switch_Mode(KTH5701_MODE_CONTINUOUS))
        {
            kth5701_ok += 1;
            break;
        }
        HAL_Delay(5);
    }
    if(kth5701_ok != 2)//kth5701_ok为2表示初始化成功
    {
        //Error_Handler();
				RED_ON();
    }
}


