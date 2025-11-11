/**
*****************************************************************************************
 * @file     msuart.h
 * @author   Michael Zhou
 * 
 * @brief    Header file of msuart.c
 * 
 * @date     2025-03-27
 * @version  0.1.1
 * 
 * @note     
 * @warning  
 * 
 * Copyright (c) WUJITECH 2019-2025. All rights reserved.
*****************************************************************************************
*/

/**************************************************************************************************
---- Define to Prevent Recursive Inclusion ------------------------------------------------------*/

#ifndef MSUART_H
#define MSUART_H

#ifdef __cplusplus
    extern "C" {
#endif

/**************************************************************************************************
---- Includes -----------------------------------------------------------------------------------*/

#include "msuart_cfg.h"
#include "main.h"

#include <stdbool.h>


/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/

/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/

// 宏函数  #define GEN_ARRAY(x)  int a[x] = {}

/**************************************************************************************************
---- Type Defines -------------------------------------------------------------------------------*/

// 定义类型  typedef unsigned char u8;

/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

typedef enum
{
    MSTR_ALL_OK                     = 0,
    MSTR_NOT_ENABLE                 = 1,
    MSTR_PARA_ERROR                 = 1 << 1,
    MSTR_TX_FAILED                  = 1 << 2,
    MSTR_RX_FAILED                  = 1 << 3,
    MSTR_TX_FAILED_AND_RX_FAILED    = MSTR_TX_FAILED | MSTR_RX_FAILED

} MSUART_Start_Transfer_Result;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

typedef struct _MSUART MSUART;

/**
 * @brief   MS UART Structure
 * @ingroup
*/
typedef struct _MSUART
{
    void                                        *args;
    UART_HandleTypeDef							*huart;             ///<pointer of the bound UART Handler
    const uint32_t								baudrate;           ///<baudrate
    const bool									hw_rs485;           ///<enable/disable hardware RS485 TX/RX control
	volatile uint16_t							tx_size;            ///<total transmit size
    volatile uint16_t							tx_count;		    ///<remaining transmit size
	volatile uint16_t							rx_size;		    ///<total receive size
	volatile uint16_t							rx_count;		    ///<remaining receive size
	
	bool										enable;             ///<enable/disable this UART Master
    volatile bool                               tx_busy;
    volatile bool                               rx_busy;
	
    void (*rx_idle_callback)(MSUART* this);
    void (*tx_cplt_callback)(MSUART* this);

    void (*sw_rs485_tx2rx)(MSUART* this);
    void (*sw_rs485_rx2tx)(MSUART* this);

} MSUART;

/**************************************************************************************************
---- Extern Variables ---------------------------------------------------------------------------*/

#if USART1_ENABLE
    extern MSUART msuart1;
#endif
#if USART2_ENABLE
    extern MSUART msuart2;
#endif
#if USART3_ENABLE
    extern MSUART msuart3; 
#endif
#if UART4_ENABLE
    extern MSUART msuart4;
#endif
#if UART5_ENABLE
    extern MSUART msuart5;
#endif
#if USART6_ENABLE
    extern MSUART msuart6;
#endif
#if UART7_ENABLE
    extern MSUART msuart7;
#endif
#if UART8_ENABLE
    extern MSUART msuart8;
#endif

/**************************************************************************************************
---- Function Prototypes ------------------------------------------------------------------------*/

static inline void MSUART_Disable(MSUART* msu)
{
	__HAL_UART_DISABLE(msu->huart);
	msu->enable = false;
}

static inline void MSUART_Enable(MSUART* msu)
{
	__HAL_UART_ENABLE(msu->huart);
	msu->enable = true;
}

static inline bool MSUART_Is_Busy(MSUART *msu)  // Hardware Busy
{
    UART_HandleTypeDef* huart = msu->huart;
    uint32_t isrflags = READ_REG(huart->Instance->ISR);

    if (msu->tx_busy && (isrflags & USART_ISR_TC) && (isrflags & USART_ISR_TXE_TXFNF))
    {
        if (!msu->hw_rs485)
        {
            if (msu->sw_rs485_tx2rx)
                msu->sw_rs485_tx2rx(msu);
        }
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);
        msu->tx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmatx);
        msu->tx_busy = false;
    }
    if (msu->rx_busy && (isrflags & USART_ISR_IDLE) && !(isrflags & USART_ISR_BUSY))
    {
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        msu->rx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
        msu->rx_busy = false;
    }
	return msu->tx_busy || msu->rx_busy;
}



void MSUART_Init(void);

MSUART_Start_Transfer_Result MSUART_Start_DMA_Transfer(MSUART *msu, bool tx, bool force_tx, void (*tx_cplt_callback)(MSUART*), const uint8_t *tx_buffer, const uint16_t tx_size, 
                                                                    bool rx, bool force_rx, bool continue_rx, void (*rx_idle_callback)(MSUART*), uint8_t *rx_buffer, const uint16_t buffer_size);

void MSUART_Abort_DMA_Transfer(MSUART *msu, bool clear_count);

void MSUART_Disable_Transmit(MSUART *msu);


/*-----------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MSUART_H */

/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/
