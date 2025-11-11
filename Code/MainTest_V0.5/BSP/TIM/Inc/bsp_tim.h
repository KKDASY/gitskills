#ifndef __SYS_TIMER_H
#define __SYS_TIMER_H

#include "stm32g4xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include "tim.h"

#define SYSTEM_TIMER_FREQ 1000000
#define SYSTEM_TIMER_PERIOD 1000

#define TIMEOUT_TIMER_FREQ 1000000
#define TIMEOUT_TIMER_PERIOD 1

#define LED_TIMER_FREQ 100000
#define LED_TIMER_PERIOD 0.5

#define GEN_TIMER_UP_IRQHANDLER(htim_ptr, irq_handler, callback) \
void irq_handler(void) \
{ \
    uint32_t itflag = READ_REG(htim_ptr->Instance->SR); \
    uint32_t itsource = READ_REG(htim_ptr->Instance->DIER); \
    if((itflag & TIM_FLAG_UPDATE) && (itsource & TIM_IT_UPDATE)) \
    { \
        __HAL_TIM_CLEAR_FLAG(htim_ptr, TIM_FLAG_UPDATE); \
        callback; \
    } \
}

typedef struct
{
    uint32_t tick_start;
    uint32_t tick_end;
    bool waiting;
    bool timeout;
}Timeout_t;

void SystemTimer_Init(void);
uint32_t SystemTimer_GetTick_ms(void);
void SystemTimer_Delay_ms(uint32_t delay);
uint32_t SystemTimer_GetElapsedTime_ms(uint32_t previous_ms);

#endif



