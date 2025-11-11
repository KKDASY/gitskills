#include "bsp_tim.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_rcc.h"
#include <stdbool.h>
#include <stdint.h>
#include "bsp_uart.h"

extern TestItem_t result_feedback;
extern UART_Buffers uart_buffers;

//static TIM_HandleTypeDef *sys_htim_ptr = &htim2;
//static TIM_TypeDef *sys_timx_ptr = TIM2;

//static TIM_HandleTypeDef *timeout_htim_ptr = &htim3;
//static TIM_TypeDef *timeout_timx_ptr = TIM3;

static TIM_HandleTypeDef *led_htim_ptr = &htim4;
static TIM_TypeDef *led_timx_ptr = TIM4;

volatile uint32_t Tick_ks = 0;
Timeout_t timeout_manager = {0};

void SYS_IncTick(void) { Tick_ks++; }

uint32_t counter = 0;

void Timeout_Handler(void)
{
	counter++;
    ID_e dst_id = ID_BROADCAST;
    TestItem_t temp = {.raw_value = 0};
    uint16_t len = Pack(dst_id, (uint8_t*)uart_buffers.tx_dma_buffer, true, temp, temp);
    MSUART_Start_DMA_Transfer(&msuart3, true, true, NULL, (const uint8_t*)uart_buffers.tx_dma_buffer, len, true, true, false, UART_RxCallback, (uint8_t*)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
}

void LED_Handler(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
}

//GEN_TIMER_UP_IRQHANDLER(sys_htim_ptr, TIM2_IRQHandler, SYS_IncTick());
//GEN_TIMER_UP_IRQHANDLER(timeout_htim_ptr, TIM3_IRQHandler, Timeout_Handler());
GEN_TIMER_UP_IRQHANDLER(led_htim_ptr, TIM4_IRQHandler, LED_Handler());

void Timer_DeInit(TIM_HandleTypeDef *htim) { HAL_TIM_Base_DeInit(htim); }

void Timer_Start(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Start(htim); }

void Timer_Stop(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Stop(htim); }

void Timer_Start_IT(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Start_IT(htim); }

void Timer_Stop_IT(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Stop_IT(htim); }

static uint32_t Get_Timer_Clock(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock = 0;
    if((uint32_t)(htim->Instance) >= APB2PERIPH_BASE)
    {
        timer_clock = HAL_RCC_GetPCLK2Freq();
        switch (RCC->CFGR & RCC_CFGR_PPRE2)
        {
        case RCC_CFGR_PPRE2_DIV4:
            timer_clock /= 2;
            break;
        case RCC_CFGR_PPRE2_DIV8:
            timer_clock /= 4;
            break;
        case RCC_CFGR_PPRE2_DIV16:
            timer_clock /= 8;
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
    HAL_NVIC_DisableIRQ((IRQn_Type)Get_Timer_Update_IRQn(htim));

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
        HAL_NVIC_EnableIRQ((IRQn_Type)Get_Timer_Update_IRQn(htim));
    }
}

void SystemTimer_Init(void)
{
//    //systim freq = 1000000 Hz, irq interval = 1000 s
//    Timer_Init(sys_htim_ptr, sys_timx_ptr, Get_Timer_Clock(sys_htim_ptr) / SYSTEM_TIMER_FREQ - 1, SYSTEM_TIMER_FREQ * SYSTEM_TIMER_PERIOD - 1, true);
//    Timer_Start_IT(sys_htim_ptr);

//    //timeout freq = 1000000 Hz, irq interval = 1 s
//    Timer_Init(timeout_htim_ptr, timeout_timx_ptr, Get_Timer_Clock(timeout_htim_ptr) / TIMEOUT_TIMER_FREQ - 1, TIMEOUT_TIMER_FREQ * TIMEOUT_TIMER_PERIOD - 1, true);
//    Timer_Start_IT(timeout_htim_ptr);

    //led freq = 100000 Hz, irq interval = 0.5 s
    Timer_Init(led_htim_ptr, led_timx_ptr, Get_Timer_Clock(led_htim_ptr) / LED_TIMER_FREQ - 1, LED_TIMER_FREQ * LED_TIMER_PERIOD - 1, true);
    Timer_Start_IT(led_htim_ptr);

    Tick_ks = 0;
}

//void BSP_TIM_Init(void)
//{
//	HAL_TIM_Base_Start_IT(&htim4);
//	HAL_Delay(10);
//	HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_4);
//	
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

//	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
//	__HAL_DBGMCU_FREEZE_TIM1();
//	__HAL_TIM_MOE_ENABLE(&htim1);

//	TIM1->CCR1 = 0;
//	TIM1->CCR2 = 0;
//	TIM1->CCR3 = 0;

//	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
//}

//uint32_t SystemTimer_GetTick_s(void)
//{
//    uint32_t return_value = 0;
//    return_value = ((uint64_t)Tick_ks * 1000 + sys_htim_ptr->Instance->CNT / SYSTEM_TIMER_FREQ) & 0xFFFFFFFF;
//    return return_value;
//}

//uint32_t SystemTimer_GetTick_ms(void)
//{
//    uint32_t return_value = 0;
//    return_value = ((uint64_t)Tick_ks * 1000000 + sys_htim_ptr->Instance->CNT / (SYSTEM_TIMER_FREQ / 1000)) & 0xFFFFFFFF;
//    return return_value;
//}

//void SystemTimer_Delay_ms(uint32_t delay)
//{
//    uint32_t start_tick = SystemTimer_GetTick_ms();
//    while(SystemTimer_GetElapsedTime_ms(start_tick) < delay)
//    {
//        __NOP();
//    }
//}

//uint32_t SystemTimer_GetElapsedTime_s(uint32_t previous_s)
//{
//    uint32_t current_s = SystemTimer_GetTick_s();
//    return current_s - previous_s;
//}

//uint32_t SystemTimer_GetElapsedTime_ms(uint32_t previous_ms)
//{
//    uint32_t current_ms = SystemTimer_GetTick_ms();
//    uint32_t elapsed_time = current_ms >= previous_ms ? current_ms - previous_ms : 0xFFFFFFFF - previous_ms + current_ms + 1;
//    return elapsed_time;
//}

//uint32_t SystemTimer_GetElapsedTime_us(uint32_t previous_us)
//{
//    uint32_t current_us = HAL_GetTick();
//    uint32_t elapsed_time = current_us >= previous_us ? current_us - previous_us : 0xFFFFFFFF - previous_us + current_us + 1;
//    return elapsed_time;
//}


