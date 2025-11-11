#include "bsp_tim.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_rcc.h"
#include <stdbool.h>
#include <stdint.h>

#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_tps26400.h"

static TIM_HandleTypeDef *sys_htim_ptr = &htim2;
static TIM_TypeDef *sys_timx_ptr = TIM2;

static TIM_HandleTypeDef *timeout_htim_ptr = &htim8;
static TIM_TypeDef *timeout_timx_ptr = TIM8;

static TIM_HandleTypeDef *led_htim_ptr = &htim1;
static TIM_TypeDef *led_timx_ptr = TIM1;

volatile uint32_t Tick_ks = 0;
extern Timeout_t timeout_manager[ID_LEN];
extern RGBState_t led_state;

void SYS_IncTick(void) { Tick_ks++; }
void TimeoutTimer_UP_IRQHandler()
{
    for(int i = 0; i < ID_LEN; i++)
    {
        if(timeout_manager[i].tick_end < SystemTimer_GetTick_ms() && timeout_manager[i].waiting)
        {
           timeout_manager[i].waiting = false;
           RED_ON();
           FLASH_ON;
	       //TPS26400_Disable();
           return;
        }
    }
    return;
}

void LEDTimer_UP_IRQHandler()
{
    LED_RGB_Controller(&led_state);
    return;
}

GEN_TIMER_UP_IRQHANDLER(sys_htim_ptr, TIM2_IRQHandler, SYS_IncTick());
GEN_TIMER_UP_IRQHANDLER(timeout_htim_ptr, TIM8_UP_IRQHandler, TimeoutTimer_UP_IRQHandler());
GEN_TIMER_UP_IRQHANDLER(led_htim_ptr, TIM1_UP_TIM16_IRQHandler, LEDTimer_UP_IRQHandler());

void Timer_Start_IT(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Start_IT(htim); }

static uint32_t Get_Timer_Clock(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock = 0;
    if((uint32_t)(htim->Instance) >= APB2PERIPH_BASE)//判断TIMx在APB2总线上
    {
        //获取PCLK2频率
        timer_clock = HAL_RCC_GetPCLK2Freq();
        //判断APB2总线是否分频并修正定时器时钟
        //解释：取CFGR寄存器中RCC_CFGR_PPRE2位置的值，该值表示APB2分频系数
        //tim分频系数 = APB2分频系数/2    例如RCC_CFGR_PPRE2_DIV4对应tim分频系数为2
        switch (RCC->CFGR & RCC_CFGR_PPRE2)
        {
        case RCC_CFGR_PPRE2_DIV4:
            timer_clock *= 2;
            break;
        case RCC_CFGR_PPRE2_DIV8:
            timer_clock *= 4;
            break;
        case RCC_CFGR_PPRE2_DIV16:
            timer_clock *= 8;
            break;
        default:
          break;
        }
    }
    else//判断TIMx在APB1总线上
    {
        //获取PCLK1频率
        timer_clock = HAL_RCC_GetPCLK1Freq();//这里获取的应该是HCLK的频率？？？写错了吧！！！
        //判断APB1总线是否分频并修正定时器时钟
        //解释：取CFGR寄存器中RCC_CFGR_PPRE1位置的值，该值表示APB1分频系数
        //tim分频系数 = APB1分频系数/2    例如RCC_CFGR_PPRE1_DIV4对应tim分频系数为2
        switch (RCC->CFGR & RCC_CFGR_PPRE1)
        {
        case RCC_CFGR_PPRE1_DIV4:
            timer_clock *= 2;
            break;
        case RCC_CFGR_PPRE1_DIV8:
            timer_clock *= 4;
            break;
        case RCC_CFGR_PPRE1_DIV16:
            timer_clock *= 8;
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

static void Timer_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIMx, uint16_t Prescaler, uint32_t Period, bool enable_irq)
{
    HAL_NVIC_DisableIRQ((IRQn_Type)Get_Timer_Update_IRQn(htim));//关闭，但这里只有TIM2中断

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

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

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

    if (enable_irq) 
    {
        HAL_NVIC_EnableIRQ((IRQn_Type)Get_Timer_Update_IRQn(htim));//这里只开启TIM2中断，而TIM8和TIM1在HAL_TIM_Base_MspInit中进行配置
    }
}

void SystemTimer_Init(void)
{
    //TIM2   systim freq = 1000000 Hz, irq interval = 1000 s  
    //1.初始化Timer
    Timer_Init(sys_htim_ptr, sys_timx_ptr, Get_Timer_Clock(sys_htim_ptr) / SYSTEM_TIMER_FREQ - 1, SYSTEM_TIMER_FREQ * SYSTEM_TIMER_PERIOD - 1, true);
    //2.开启中断
    Timer_Start_IT(sys_htim_ptr);
    
    //TIM8   TIMEOUT_TIMER_FREQ 1000000Hz    TIMEOUT_TIMER_PERIOD 1 s
    Timer_Init(timeout_htim_ptr, timeout_timx_ptr, Get_Timer_Clock(timeout_htim_ptr) / TIMEOUT_TIMER_FREQ - 1, TIMEOUT_TIMER_FREQ * TIMEOUT_TIMER_PERIOD - 1, true);
    Timer_Start_IT(timeout_htim_ptr);
    
    //TIM1   LED_TIMER_FREQ 100000      LED_TIMER_PERIOD 0.5s
    Timer_Init(led_htim_ptr, led_timx_ptr, Get_Timer_Clock(led_htim_ptr) / LED_TIMER_FREQ - 1, LED_TIMER_FREQ * LED_TIMER_PERIOD - 1, true);
    Timer_Start_IT(led_htim_ptr);
    
    Tick_ks = 0;
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

uint32_t SystemTimer_GetElapsedTime_ms(uint32_t previous_ms)
{
    uint32_t current_ms = SystemTimer_GetTick_ms();
    uint32_t elapsed_time = current_ms >= previous_ms ? current_ms - previous_ms : 0xFFFFFFFF - previous_ms + current_ms + 1;
    return elapsed_time;
}
