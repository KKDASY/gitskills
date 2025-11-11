#include "bsp_tim.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_rcc.h"
#include <stdbool.h>
#include <stdint.h>
#include "bsp_uart.h"

extern TestItem_t result_feedback;

static TIM_HandleTypeDef *sys_htim_ptr = &htim2;
static TIM_TypeDef *sys_timx_ptr = TIM2;

static TIM_HandleTypeDef *timeout_htim_ptr = &htim3;
static TIM_TypeDef *timeout_timx_ptr = TIM3;

static TIM_HandleTypeDef *led_htim_ptr = &htim4;
static TIM_TypeDef *led_timx_ptr = TIM4;

volatile uint32_t Tick_ks = 0;
Timeout_t timeout_manager = {0};

//系统主定时器回调函数：Tick_ks++
void SYS_IncTick(void) { Tick_ks++; }

//超时定时器回调函数：根据设置的timeout_manager.tick_end（定时达到时间）和timeout_manager.waiting（等待状态是是否为true）
void Timeout_Handler(void)
{
    //到达超时时间且处于等待状态
    if(timeout_manager.tick_end < SystemTimer_GetTick_ms() && timeout_manager.waiting)
    {
        timeout_manager.timeout = true;//超时
        timeout_manager.waiting = false;//取消等待

        result_feedback.bits.Item_RS485TxRx = 0;//将RS485通信结果反馈标志位清零，表示通信失败或超时
    }
}
//LED定时器回调函数：LED翻转
void LED_Handler(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}
//功能：定义中断函数
//给sys_htim_ptr定时器句柄的中断TIM2_IRQHandler配置回调函数SYS_IncTick
GEN_TIMER_UP_IRQHANDLER(sys_htim_ptr, TIM2_IRQHandler, SYS_IncTick());
GEN_TIMER_UP_IRQHANDLER(timeout_htim_ptr, TIM3_IRQHandler, Timeout_Handler());
GEN_TIMER_UP_IRQHANDLER(led_htim_ptr, TIM4_IRQHandler, LED_Handler());

void Timer_DeInit(TIM_HandleTypeDef *htim) { HAL_TIM_Base_DeInit(htim); }

void Timer_Start(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Start(htim); }

void Timer_Stop(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Stop(htim); }

void Timer_Start_IT(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Start_IT(htim); }

void Timer_Stop_IT(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Stop_IT(htim); }
/*
获取定时器实际时钟频率的函数
1.timer_clock初始化
2.判断tim所在总线
3.根据总线获取总线频率
4.根据总线分频系数获取TIM分频系数
5.timer_clock返回
*/
static uint32_t Get_Timer_Clock(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock = 0;//存储计算的得到的时钟频率
    //判断定时器属于APB2总线还是APB1总线
    //APB2PERIPH_BASE是APB2外设的基地址
    if((uint32_t)(htim->Instance) >= APB2PERIPH_BASE)
    {
        //获取APB2总线时钟频率（PCLK2）
        timer_clock = HAL_RCC_GetPCLK2Freq();
        //检查APB2预分频系数
        switch (RCC->CFGR & RCC_CFGR_PPRE2)
        {
        case RCC_CFGR_PPRE2_DIV4://如果预分频是4分频
            timer_clock /= 2;//实际定时器时钟要/2
            break;
        case RCC_CFGR_PPRE2_DIV8://如果预分频是8分频
            timer_clock /= 4;//实际定时器时钟要/4
            break;
        case RCC_CFGR_PPRE2_DIV16://如果预分频是16分频
            timer_clock /= 8;//实际定时器时钟要/8
            break;
        default:
          break;
        }
    }
    else
    {
        timer_clock = HAL_RCC_GetPCLK1Freq();
        switch (RCC->CFGR & RCC_CFGR_PPRE1)
        {
        case RCC_CFGR_PPRE1_DIV4:
            timer_clock /= 2;
            break;
        case RCC_CFGR_PPRE1_DIV8:
            timer_clock /= 4;
            break;
        case RCC_CFGR_PPRE1_DIV16:
            timer_clock /= 8;
            break;
        default:
            break;
        }
    }
    return timer_clock;
}

static uint32_t Get_Timer_Update_IRQn(TIM_HandleTypeDef *htim)
{
	uint32_t return_val = 0;
    if(htim->Instance == TIM2)
    {
        return TIM2_IRQn;
    }
	return return_val;
}

void Timer_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIMx, uint16_t Prescaler, uint32_t Period, bool enable_irq)
{
    //1.禁用定时器更新中断（但Get_Timer_Update_IRQn函数只能获取TIM2的中断）
    HAL_NVIC_DisableIRQ(Get_Timer_Update_IRQn(htim));
    
    //2.时基单元配置及初始化
    htim->Instance = TIMx;
    htim->Init.Prescaler = Prescaler;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = Period;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;

    if(HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
    //3.配置定时器时钟源（使用内部时钟）
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    //4.配置主从模式（用于定时器同步）
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if(htim->Instance == TIM8 || htim->Instance == TIM1)
    {
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    }
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) 
    {
        Error_Handler();
    }
    
    //5.按需使能定时器更新中断
    if (enable_irq) 
    {
        HAL_NVIC_EnableIRQ(Get_Timer_Update_IRQn(htim));//！！目前Get_Timer_Update_IRQn函数只支持获取TIM2中断，其它需要补充
    }
}
//系统定时器初始化函数，设置多久触发一次中断
//sys_timx_ptr      -- tim2   1000s/次  Tick_ks++
//timeout_timx_ptr  -- tim3   0.2s/次   判断超时（设置超时标志）
//led_timx_ptr      -- tim4   0.5s/次   led翻转
void SystemTimer_Init(void)
{
    //systim freq = 1000000 Hz, 溢出时间irq interval = 1000 s
    //系统主定时器初始化函数
    Timer_Init(sys_htim_ptr, sys_timx_ptr, Get_Timer_Clock(sys_htim_ptr) / SYSTEM_TIMER_FREQ - 1, SYSTEM_TIMER_FREQ * SYSTEM_TIMER_PERIOD - 1, true);
    Timer_Start_IT(sys_htim_ptr);//开启时基单元

    //timeout freq = 1000000 Hz, 溢出时间irq interval = 0.2 s
    //超时检测定时器初始化
    Timer_Init(timeout_htim_ptr, timeout_timx_ptr, Get_Timer_Clock(timeout_htim_ptr) / TIMEOUT_TIMER_FREQ - 1, TIMEOUT_TIMER_FREQ * TIMEOUT_TIMER_PERIOD - 1, true);
    Timer_Start_IT(timeout_htim_ptr);

    //led freq = 100000 Hz, 溢出时间irq interval = 0.5 s
    //LED闪烁定时器初始化
    Timer_Init(led_htim_ptr, led_timx_ptr, Get_Timer_Clock(led_htim_ptr) / LED_TIMER_FREQ - 1, LED_TIMER_FREQ * LED_TIMER_PERIOD - 1, true);
    Timer_Start_IT(led_htim_ptr);

    Tick_ks = 0;
}

void BSP_TIM_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim4);//？？？这里的意义是？？？
	HAL_Delay(10);
	HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//PA8
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//PA9
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//PA10

	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);//禁用tim1的主输出使能（MOE），高级定时器的保护机制，防止意外输出
	__HAL_DBGMCU_FREEZE_TIM1();//调试模式下冻结TIM1计数器，当芯片被调试器暂停时，TIM1也会停止计数
	__HAL_TIM_MOE_ENABLE(&htim1);//重新使能TIM1主输出

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
}

uint32_t SystemTimer_GetTick_s(void)
{
    uint32_t return_value = 0;
    return_value = ((uint64_t)Tick_ks * 1000 + sys_htim_ptr->Instance->CNT / SYSTEM_TIMER_FREQ) & 0xFFFFFFFF;
    return return_value;
}

uint32_t SystemTimer_GetTick_ms(void)
{
    uint32_t return_value = 0;
    return_value = ((uint64_t)Tick_ks * 1000000 + sys_htim_ptr->Instance->CNT / (SYSTEM_TIMER_FREQ / 1000)) & 0xFFFFFFFF;
    return return_value;
}

void SystemTimer_Delay_ms(uint32_t delay)
{
    uint32_t start_tick = SystemTimer_GetTick_ms();
    while(SystemTimer_GetElapsedTime_ms(start_tick) < delay)
    {
        __NOP();
    }
}

uint32_t SystemTimer_GetElapsedTime_s(uint32_t previous_s)
{
    uint32_t current_s = SystemTimer_GetTick_s();
    return current_s - previous_s;
}

uint32_t SystemTimer_GetElapsedTime_ms(uint32_t previous_ms)
{
    uint32_t current_ms = SystemTimer_GetTick_ms();
    uint32_t elapsed_time = current_ms >= previous_ms ? current_ms - previous_ms : 0xFFFFFFFF - previous_ms + current_ms + 1;
    return elapsed_time;
}

uint32_t SystemTimer_GetElapsedTime_us(uint32_t previous_us)
{
    uint32_t current_us = HAL_GetTick();
    uint32_t elapsed_time = current_us >= previous_us ? current_us - previous_us : 0xFFFFFFFF - previous_us + current_us + 1;
    return elapsed_time;
}
