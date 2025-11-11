/**************************************************************************************************
---- Define to Prevent Recursive Inclusion ------------------------------------------------------*/

#ifndef MSUART_CFG_H
#define MSUART_CFG_H

#ifdef __cplusplus
    extern "C" {
#endif

/**************************************************************************************************
---- Includes -----------------------------------------------------------------------------------*/


/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/

/* User Configurations */

#define USART1_ENABLE   0
#if USART1_ENABLE
    #define MSUART1_BAUDRATE     BRAIN_MSUART_BAUDRATE
    #define MSUART1_HW_RS485_ENABLE    1
    #define MSUART1_TX_DMA_NUM         2
    #define MSUART1_RX_DMA_NUM         2
    #define MSUART1_TX_DMA_STREAM_NUM  1
    #define MSUART1_RX_DMA_STREAM_NUM  0
#endif

#define USART2_ENABLE   0
#if USART2_ENABLE
    #define MSUART2_BAUDRATE     FINGER_MSUART_BAUDRATE
    #define MSUART2_HW_RS485_ENABLE    1
    #define MSUART2_TX_DMA_NUM         1
    #define MSUART2_RX_DMA_NUM         1
    #define MSUART2_TX_DMA_STREAM_NUM  1
    #define MSUART2_RX_DMA_STREAM_NUM  0
#endif

#define USART3_ENABLE   1
#if USART3_ENABLE
    #define MSUART3_BAUDRATE     921600
    #define MSUART3_HW_RS485_ENABLE    1
    #define MSUART3_TX_DMA_NUM         1
    #define MSUART3_RX_DMA_NUM         1
    #define MSUART3_TX_DMA_STREAM_NUM  2
    #define MSUART3_RX_DMA_STREAM_NUM  1
#endif

#define UART4_ENABLE    0
#if UART4_ENABLE
    #define MSUART4_BAUDRATE     BRAIN_MSUART_BAUDRATE
    #define MSUART4_HW_RS485_ENABLE    1
    #define MSUART4_TX_DMA_NUM         2
    #define MSUART4_RX_DMA_NUM         2
    #define MSUART4_TX_DMA_STREAM_NUM  1
    #define MSUART4_RX_DMA_STREAM_NUM  0
#endif

#define UART5_ENABLE    0
#if UART5_ENABLE
    #define MSUART5_BAUDRATE    FINGER_MSUART_BAUDRATE
    #define MSUART5_HW_RS485_ENABLE    1
    #define MSUART5_TX_DMA_NUM         1
    #define MSUART5_RX_DMA_NUM         1
    #define MSUART5_TX_DMA_STREAM_NUM  5
    #define MSUART5_RX_DMA_STREAM_NUM  4
#endif

#define USART6_ENABLE   0
#if USART6_ENABLE
    #define MSUART6_BAUDRATE    1000000
    #define MSUART6_HW_RS485_ENABLE    0
    #define MSUART6_TX_DMA_NUM         1
    #define MSUART6_RX_DMA_NUM         1
    #define MSUART6_TX_DMA_STREAM_NUM  1
    #define MSUART6_RX_DMA_STREAM_NUM  0
#endif

#define UART7_ENABLE    0
#if UART7_ENABLE
    #define MSUART7_BAUDRATE    FINGER_MSUART_BAUDRATE
    #define MSUART7_HW_RS485_ENABLE    1
    #define MSUART7_TX_DMA_NUM         2
    #define MSUART7_RX_DMA_NUM         2
    #define MSUART7_TX_DMA_STREAM_NUM  3
    #define MSUART7_RX_DMA_STREAM_NUM  2
#endif

#define UART8_ENABLE    0
#if UART8_ENABLE
    #define MSUART8_BAUDRATE    FINGER_MSUART_BAUDRATE
    #define MSUART8_HW_RS485_ENABLE    1
    #define MSUART8_TX_DMA_NUM         2
    #define MSUART8_RX_DMA_NUM         2
    #define MSUART8_TX_DMA_STREAM_NUM  5
    #define MSUART8_RX_DMA_STREAM_NUM  4
#endif

/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/

// 宏函数  #define GEN_ARRAY(x)  int a[x] = {}

/**************************************************************************************************
---- Type Defines -------------------------------------------------------------------------------*/

// 定义类型  typedef unsigned char u8;

/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

// 枚举  typedef enum { OK, FAILED, ERROR } Status;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

// 结构体  typedef struct { int a; float b; char *c; } Packet;

/**************************************************************************************************
---- Extern Variables ---------------------------------------------------------------------------*/

// 接口全局变量 extern int global_var;

/**************************************************************************************************
---- Function Prototypes ------------------------------------------------------------------------*/

// 接口函数原型 (extern) void func(void);

/*-----------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MSUART_CFG_H */

/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/


