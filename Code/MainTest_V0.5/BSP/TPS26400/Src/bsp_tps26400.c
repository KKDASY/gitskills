#include "bsp_tps26400.h"
#include "bsp_tim.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_kth5701.h"

extern RGBState_t led_state;
extern TestedObj_e TestedObj_Current;
bool TPS26400_Async_Enable_Signal = false;

static void TPS26400_Enable(void);
    
static void TPS26400_Enable(void) 
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

void TPS26400_Disable(void) 
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

void TPS26400_Delay_Enable_Signal(void) 
{ 
    TPS26400_Async_Enable_Signal = true; 
}
/**
* 用于电源管理芯片（TPS26400，过流保护开关）的延时启动，启动后绿灯闪烁
* 如果测试对象是MAWire（主线缆）还需要初始化KTH5701
*/
void TPS26400_Delay_Enable(void) 
{
    //如果“异步使能信号”为true  由key4触发
    if (TPS26400_Async_Enable_Signal) 
    {
        SystemTimer_Delay_ms(500);
        TPS26400_Enable();//使能TPS26400芯片（硬件层面开启电源输出）
        SystemTimer_Delay_ms(1000);
        TPS26400_Async_Enable_Signal = false;//防止重复使能

        if(TestedObj_Current == TestedObj_MAWire)//如果测试对象是MAWire（带磁编排线）
        {
            KTH5701_BSP_Init();//初始化KTH5701
        }
				
				GREEN_ON();//绿灯
        FLASH_ON;//闪烁
    }
}

