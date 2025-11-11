/**
*****************************************************************************************
 * @file     msuart.c
 * @author   Michael Zhou
 * 
 * @brief    High-speed real-time UART driver for real-time using in the system
 *           + Suitable for half-duplex communication scenarios
 *           + Each UART utilizes hardware FIFO and DMA to reduce CPU and bus usage
 *           + Highly optimized for minimal system overhead and additional latency
 * 
 * @date     2025-03-31
 * @version  0.1.2
 * 
 * @note     
 * @warning
 * 
 * Copyright (c) WUJITECH 2019-2025. All rights reserved.
*****************************************************************************************
*/

/**************************************************************************************************
---- Includes -----------------------------------------------------------------------------------*/

#include "msuart.h"

/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/



/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/

#if defined(STM32H7)
#define GET_DMA_STREAM_IRQN(dma, stream)    DMA##dma##_Stream##stream##_IRQn
#elif defined(STM32G4)
#define GET_DMA_STREAM_IRQN(dma, stream)    DMA##dma##_Channel##stream##_IRQn
#endif

#define DISABLE_DMA_IRQ(dma, stream)        __NVIC_DisableIRQ(GET_DMA_STREAM_IRQN(dma, stream))
#define DISABLE_UARTX_RX_DMA_IRQ(x)         DISABLE_DMA_IRQ(MSUART##x##_RX_DMA_NUM, MSUART##x##_RX_DMA_STREAM_NUM)
#define DISABLE_UARTX_TX_DMA_IRQ(x)         DISABLE_DMA_IRQ(MSUART##x##_TX_DMA_NUM, MSUART##x##_TX_DMA_STREAM_NUM)

// #define UARTX_TX_DMA_IRQHANDLER(x, dma, stream)   void DMA##dma##_Stream##stream##_IRQHandler(void)  { MSUART_TX_DMA_IRQ_Handler(&msuart##x); }
// #define GEN_UARTX_TX_DMA_IRQHANDLER(x)    UARTX_TX_DMA_IRQHANDLER(x, MSUART##x##_TX_DMA_NUM, MSUART##x##_TX_DMA_STREAM_NUM)

#define GEN_MSUARTX(x)                      MSUART msuart##x = { .huart = &huart##x, .baudrate = MSUART##x##_BAUDRATE, .hw_rs485 = MSUART##x##_HW_RS485_ENABLE, \
															    .tx_size = 0, .rx_size = 0, .rx_count = 0, \
															    .enable = true, .tx_busy = false, .rx_busy = false, \
															    .rx_idle_callback = NULL, .tx_cplt_callback = NULL };
 
#define UARTX_IRQ_HANDLER_BODY(x)           { MSUART_IRQ_Handler(&msuart##x); }
#define GEN_UARTX_IRQHANDLER(x)             void UART##x##_IRQHandler(void)  UARTX_IRQ_HANDLER_BODY(x)
#define GEN_USARTX_IRQHANDLER(x)            void USART##x##_IRQHandler(void) UARTX_IRQ_HANDLER_BODY(x)

/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

// 枚举  typedef enum { OK, FAILED, ERROR } Status;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

// 结构体  typedef struct { int a; float b; char *c; } Packet;

/**************************************************************************************************
---- Constants ----------------------------------------------------------------------------------*/

// 常量  const int x = 1000;
															   
/**************************************************************************************************
---- Private Function Prototypes ----------------------------------------------------------------*/
														   
static void MSUART_IRQ_Handler(MSUART *msu);

/**************************************************************************************************
---- Private Variables --------------------------------------------------------------------------*/



/**************************************************************************************************
---- Exported Variables -------------------------------------------------------------------------*/

#if USART1_ENABLE
    extern UART_HandleTypeDef huart1;
    GEN_MSUARTX(1)
    GEN_USARTX_IRQHANDLER(1)
#endif
#if USART2_ENABLE
    extern UART_HandleTypeDef huart2;
    GEN_MSUARTX(2)
    GEN_USARTX_IRQHANDLER(2)
#endif
#if USART3_ENABLE
    extern UART_HandleTypeDef huart3;
    GEN_MSUARTX(3)
    GEN_USARTX_IRQHANDLER(3)
#endif
#if UART4_ENABLE
    extern UART_HandleTypeDef huart4;
    GEN_MSUARTX(4)
    GEN_UARTX_IRQHANDLER(4)
#endif
#if UART5_ENABLE
    extern UART_HandleTypeDef huart5;
    GEN_MSUARTX(5)
    GEN_UARTX_IRQHANDLER(5)
#endif
#if USART6_ENABLE
    extern UART_HandleTypeDef huart6;
    GEN_MSUARTX(6)
    GEN_USARTX_IRQHANDLER(6)
#endif
#if UART7_ENABLE
    extern UART_HandleTypeDef huart7;
    GEN_MSUARTX(7)
    GEN_UARTX_IRQHANDLER(7)
#endif
#if UART8_ENABLE
    extern UART_HandleTypeDef huart8;
    GEN_MSUARTX(8)
    GEN_UARTX_IRQHANDLER(8)
#endif

/**************************************************************************************************
---- Private Function Implementations------------------------------------------------------------*/

static inline void DMA_Set_Config(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    /* Clear the DMAMUX synchro overrun flag */
    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
    if(hdma->DMAmuxRequestGen != 0U)
    {
        /* Clear the DMAMUX request generator overrun flag */
        hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }

#if defined(STM32H7)
    DMA_Stream_TypeDef *DMAx_Stream = (DMA_Stream_TypeDef *)hdma->Instance;
    /* Clear all interrupt flags at correct offset within the register */
    *((volatile uint32_t *)(hdma->StreamBaseAddress + 0x08)) = 0x3FUL << (hdma->StreamIndex & 0x1FU);
    /* Clear DBM bit */
    DMAx_Stream->CR &= ~DMA_SxCR_DBM;
    /* Configure DMA Stream data length */
    DMAx_Stream->NDTR = DataLength;
	
	/* Peripheral to Memory */
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
        /* Configure DMA Stream destination address */
        DMAx_Stream->PAR = DstAddress;
        /* Configure DMA Stream source address */
        DMAx_Stream->M0AR = SrcAddress;
    }
    /* Memory to Peripheral */
    else
    {
        /* Configure DMA Stream source address */
        DMAx_Stream->PAR = SrcAddress;
        /* Configure DMA Stream destination address */
        DMAx_Stream->M0AR = DstAddress;
    }

#elif defined(STM32G4)
    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = DataLength;
	
	/* Memory to Peripheral */
	if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
	{
		/* Configure DMA Channel destination address */
		hdma->Instance->CPAR = DstAddress;

		/* Configure DMA Channel source address */
		hdma->Instance->CMAR = SrcAddress;
	}
	/* Peripheral to Memory */
	else
	{
		/* Configure DMA Channel source address */
		hdma->Instance->CPAR = SrcAddress;

		/* Configure DMA Channel destination address */
		hdma->Instance->CMAR = DstAddress;
	}
#endif

}

static void UART_Generate_Instance(UART_HandleTypeDef* huart)
{
#if USART1_ENABLE
	if(huart == &huart1)	{ huart->Instance = USART1; return; }
#endif
#if USART2_ENABLE
	if(huart == &huart2)	{ huart->Instance = USART2; return; }
#endif
#if USART3_ENABLE
	if(huart == &huart3)	{ huart->Instance = USART3; return; }
#endif
#if UART4_ENABLE
	if(huart == &huart4)	{ huart->Instance = UART4; return; }
#endif
#if UART5_ENABLE
	if(huart == &huart5)	{ huart->Instance = UART5; return; }
#endif
#if USART6_ENABLE
	if(huart == &huart6)	{ huart->Instance = USART6; return; }
#endif
#if UART7_ENABLE
	if(huart == &huart7)	{ huart->Instance = UART7; return; }
#endif
#if UART8_ENABLE
	if(huart == &huart8)	{ huart->Instance = UART8; return; }
#endif
}

/**
*****************************************************************************************
 * @brief    Get DMA requeset selection number via transmission direction.
 * @param    huart      Pointer to UART Handler
 * @param    is_tx      Direction, true is transmit or false is receive
 * @return   uint32_t   DMA requeset selection number
 * 
 * @note     
 * @warning  The supported UARTx may vary between different devices.
 * 
 * @author   Michael Zhou
 * @date     2025-02-13
*****************************************************************************************
*/
static uint32_t UART_Get_DMA_Request_Selection(USART_TypeDef* uart, bool is_tx)
{
	uint32_t request_selection = 0;
	if(uart == USART1)	request_selection = is_tx ? DMA_REQUEST_USART1_TX : DMA_REQUEST_USART1_RX;
	else if(uart == USART2)	request_selection = is_tx ? DMA_REQUEST_USART2_TX : DMA_REQUEST_USART2_RX;
	else if(uart == USART3)	request_selection = is_tx ? DMA_REQUEST_USART3_TX : DMA_REQUEST_USART3_RX;

#if defined(STM32H7)
	else if(uart == UART4)	request_selection = is_tx ? DMA_REQUEST_UART4_TX : DMA_REQUEST_UART4_RX;
	else if(uart == UART5)	request_selection = is_tx ? DMA_REQUEST_UART5_TX : DMA_REQUEST_UART5_RX;
	else if(uart == USART6)	request_selection = is_tx ? DMA_REQUEST_USART6_TX : DMA_REQUEST_USART6_RX;
	else if(uart == UART7)	request_selection = is_tx ? DMA_REQUEST_UART7_TX : DMA_REQUEST_UART7_RX;
	else if(uart == UART8)	request_selection = is_tx ? DMA_REQUEST_UART8_TX : DMA_REQUEST_UART8_RX;
#endif

	return request_selection;
}

/**
*****************************************************************************************
 * @brief    UART hardware communication configuration and initialization.
 * @param    huart      Pointer to UART Handler
 * @param    baudrate   Expected baudrate
 * @param    HW_RS485   Enable/Disable hardware RS485 TX/RX control
 * @param    enable_irq Enable/Disable UART Interrupt request
 * 
 * @note     Does not support RS485 software TX/RX control.
 * @warning  
 * 
 * @author   Michael Zhou
 * @date     2025-02-13
*****************************************************************************************
*/
static void MSUART_Commu_Init(MSUART *msu)
{
    UART_HandleTypeDef* huart = msu->huart;

    /* Set Basic Config */
    UART_Generate_Instance(huart);
	huart->Init.BaudRate = msu->baudrate;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.OverSampling = msu->baudrate > 5000000 ?  UART_OVERSAMPLING_8 : UART_OVERSAMPLING_16;
	huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
	huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    /* Call HAL UART Init Function */
	if(msu->hw_rs485 == true)
	{
		if (HAL_RS485Ex_Init(huart, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
		    Error_Handler();
	}
	else
	{
		if (HAL_UART_Init(huart) != HAL_OK)
			Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
		Error_Handler();
	if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
		Error_Handler();
	if (HAL_UARTEx_EnableFifoMode(huart) != HAL_OK)
		Error_Handler();
    /* Disable IRQ & Clear Flags */
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_TXEIE | 
                                    USART_CR1_PEIE | USART_CR1_CMIE | USART_CR1_RTOIE | USART_CR1_EOBIE |
                                    USART_CR1_TXFEIE | USART_CR1_RXFFIE);
    CLEAR_BIT(huart->Instance->CR2, USART_CR2_LBDIE);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE | USART_CR3_CTSIE | USART_CR3_WUFIE | USART_CR3_TXFTIE |  USART_CR3_RXFTIE);

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF | UART_CLEAR_IDLEF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
	/* Enable IRQ */

    uint32_t irqn = 0;
    if(huart->Instance == USART1)	    irqn = USART1_IRQn;
    else if(huart->Instance == USART2)	irqn = USART2_IRQn;
    else if(huart->Instance == USART3)	irqn = USART3_IRQn;

#if defined(STM32H7)
    else if(huart->Instance == UART4)	irqn = UART4_IRQn;
    else if(huart->Instance == UART5)	irqn = UART5_IRQn;
    else if(huart->Instance == USART6)	irqn = USART6_IRQn;
    else if(huart->Instance == UART7)	irqn = UART7_IRQn;
    else if(huart->Instance == UART8)	irqn = UART8_IRQn;
#endif

    if(irqn)
        HAL_NVIC_EnableIRQ((IRQn_Type)irqn);
}


/**
*****************************************************************************************
 * @brief    UART DMA Initialization.
 * @param    msu    Pointer to MSUART
 * 
 * @note     UART transmission and reception DMA are bound to the same DMA_Stream due to 
 *           time-division multiplexing
 * @warning  
 * 
 * @author   Michael Zhou
 * @date     2025-02-13
*****************************************************************************************
*/
static void MSUART_DMA_Init(MSUART *msu)
{
	UART_HandleTypeDef* huart = msu->huart;
	DMA_HandleTypeDef* hdma_tx = huart->hdmatx;
	DMA_HandleTypeDef* hdma_rx = huart->hdmarx;
#if defined(STM32H7)
	DMA_Stream_TypeDef* tx_stream = hdma_tx->Instance;
	DMA_Stream_TypeDef* rx_stream = hdma_rx->Instance;
#elif defined(STM32G4)
	DMA_Channel_TypeDef* tx_stream = hdma_tx->Instance;
	DMA_Channel_TypeDef* rx_stream = hdma_rx->Instance;
#endif

	if(huart == NULL || hdma_tx == NULL || hdma_rx == NULL || tx_stream == NULL || rx_stream == NULL)
		Error_Handler();

	/* TX DMA Config */
	hdma_tx->Instance = tx_stream;
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx->Init.Mode = DMA_NORMAL;
#if defined(STM32H7)
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_tx->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx->Init.MemBurst = DMA_MBURST_INC4;
    hdma_tx->Init.PeriphBurst = DMA_PBURST_SINGLE;
#elif defined(STM32G4)
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
#endif	
	hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tx->Init.Request = UART_Get_DMA_Request_Selection(huart->Instance, true);

    if(HAL_DMA_Init(hdma_tx) != HAL_OK)
		Error_Handler();
    
    /* RX DMA Config */
    hdma_rx->Instance = rx_stream;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx->Init.Mode = DMA_NORMAL;
#if defined(STM32H7)
	hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_rx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx->Init.MemBurst = DMA_MBURST_INC4;
    hdma_rx->Init.PeriphBurst = DMA_PBURST_SINGLE;
#elif defined(STM32G4)
    hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
#endif
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx->Init.Request = UART_Get_DMA_Request_Selection(huart->Instance, false);

    if(HAL_DMA_Init(hdma_rx) != HAL_OK)
        Error_Handler();

	/* Link DMA Handler to UART Handler */
    __HAL_LINKDMA(huart, hdmatx, *hdma_tx);
	__HAL_LINKDMA(huart, hdmarx, *hdma_rx);
}

static void MSUART_IRQ_Handler(MSUART *msu)
{
    UART_HandleTypeDef* huart = msu->huart;
    uint32_t isrflags   = READ_REG(huart->Instance->ISR);
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);

    /*  Transmit Complete */
    if ((isrflags & USART_ISR_TC) && (cr1its & USART_CR1_TCIE))
    {
        if (!msu->hw_rs485)
        {
            if (msu->sw_rs485_tx2rx)
                msu->sw_rs485_tx2rx(msu);
        }
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);
        msu->tx_busy = false;
        if(msu->enable == true)
        {
            msu->tx_count = __HAL_DMA_GET_COUNTER(huart->hdmatx);
            if (msu->tx_cplt_callback)
                msu->tx_cplt_callback(msu);
        }
    }
    /* Receive Idle */
    else if ((isrflags & USART_ISR_IDLE) && (cr1its & USART_CR1_IDLEIE))
    {
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        msu->rx_busy = false;
        if(msu->enable == true)
        {
            msu->rx_count = __HAL_DMA_GET_COUNTER(huart->hdmarx);
            if (msu->rx_idle_callback)
                msu->rx_idle_callback(msu);
        }
    }  
}

/**************************************************************************************************
---- Exported Function Implementations-----------------------------------------------------------*/

void MSUART_Init(void)
{
#if USART1_ENABLE
	MSUART_Commu_Init(&msuart1);
	MSUART_DMA_Init(&msuart1);
    DISABLE_UARTX_TX_DMA_IRQ(1);
    DISABLE_UARTX_RX_DMA_IRQ(1);
#endif
#if USART2_ENABLE
	MSUART_Commu_Init(&msuart2);
	MSUART_DMA_Init(&msuart2);
    DISABLE_UARTX_TX_DMA_IRQ(2);
    DISABLE_UARTX_RX_DMA_IRQ(2);
#endif
#if USART3_ENABLE
	MSUART_Commu_Init(&msuart3);
	MSUART_DMA_Init(&msuart3);
    DISABLE_UARTX_TX_DMA_IRQ(3);
    DISABLE_UARTX_RX_DMA_IRQ(3);
#endif
#if UART4_ENABLE
	MSUART_Commu_Init(&msuart4);
	MSUART_DMA_Init(&msuart4);
    DISABLE_UARTX_TX_DMA_IRQ(4);
    DISABLE_UARTX_RX_DMA_IRQ(4);
#endif
#if UART5_ENABLE
	MSUART_Commu_Init(&msuart5);
	MSUART_DMA_Init(&msuart5);
    DISABLE_UARTX_TX_DMA_IRQ(5);
    DISABLE_UARTX_RX_DMA_IRQ(5);
#endif
#if USART6_ENABLE
	MSUART_Commu_Init(&msuart6);
	MSUART_DMA_Init(&msuart6);
    DISABLE_UARTX_TX_DMA_IRQ(6);
    DISABLE_UARTX_RX_DMA_IRQ(6);
#endif
#if UART7_ENABLE
	MSUART_Commu_Init(&msuart7);
	MSUART_DMA_Init(&msuart7);
    DISABLE_UARTX_TX_DMA_IRQ(7);
    DISABLE_UARTX_RX_DMA_IRQ(7);
#endif
#if UART8_ENABLE
	MSUART_Commu_Init(&msuart8);
	MSUART_DMA_Init(&msuart8);
    DISABLE_UARTX_TX_DMA_IRQ(8);
    DISABLE_UARTX_RX_DMA_IRQ(8);
#endif
}

MSUART_Start_Transfer_Result MSUART_Start_DMA_Transfer(MSUART *msu, bool tx, bool force_tx, void (*tx_cplt_callback)(MSUART*), const uint8_t *tx_buffer, const uint16_t tx_size, 
                                                                    bool rx, bool force_rx, bool continue_rx, void (*rx_idle_callback)(MSUART*), uint8_t *rx_buffer, const uint16_t buffer_size)
{
    UART_HandleTypeDef* huart = msu->huart;
    uint8_t result = MSTR_ALL_OK;

    if (msu->enable == false)
        return MSTR_NOT_ENABLE;
    
    if ((tx && rx) && !msu->hw_rs485 && tx_cplt_callback)  // if tx and rx are both enabled, rs485 sw flow control must be switched in tx_cplt_callback manually
        return MSTR_PARA_ERROR;

    /* Update TX/RX Status */
    uint32_t isrflags = READ_REG(huart->Instance->ISR);
    if (msu->tx_busy)
    {
        /* Transmit Complete */
        if ((isrflags & USART_ISR_TC) && (isrflags & USART_ISR_TXE_TXFNF))
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);
            msu->tx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmatx);
            msu->tx_busy = false;
        }
        /* Abort Ongoing Transmission */
        else if (tx && force_tx)
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAT));
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TCIE));
            __HAL_UART_SEND_REQ(huart, UART_TXDATA_FLUSH_REQUEST);
            msu->tx_busy = false;
        }
    }
    if (msu->rx_busy)
    {
        /* Receive Idle */
        if ((isrflags & USART_ISR_IDLE) && !(isrflags & USART_ISR_BUSY))
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
            msu->rx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
            msu->rx_busy = false;
        }
        /* Abort Ongoing Reception */
        else if (rx && force_rx)
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAR));
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_IDLEIE));
            __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
            msu->rx_busy = false;
        }
    }
    
    /* Start Transmit */
    if (tx && !msu->tx_busy)
    {
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        msu->tx_size = tx_size;
        msu->tx_count = tx_size;
        __HAL_DMA_DISABLE(huart->hdmatx);
        DMA_Set_Config(huart->hdmatx, (uint32_t)(tx_buffer), (uint32_t)&(huart->Instance->TDR), tx_size);
        __HAL_DMA_ENABLE(huart->hdmatx);
        ATOMIC_SET_BIT(huart->Instance->CR1, (USART_CR1_TE));
        msu->tx_busy = true;
        if (!msu->hw_rs485)
        {
            if (msu->sw_rs485_rx2tx)
                msu->sw_rs485_rx2tx(msu);
        }
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
        if (tx_cplt_callback)
        {
            msu->tx_cplt_callback = tx_cplt_callback;
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
        }
    }
    else if (tx)
        result |= MSTR_TX_FAILED;

    /* Start Receive */
    if (rx && !msu->rx_busy)
    {
				__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        
        if (!continue_rx)
        {
						/* Start New Receive */
						msu->rx_size = buffer_size;
						msu->rx_count = buffer_size;
						uint32_t dummy = huart->Instance->RDR;
						(void)dummy;

						__HAL_DMA_DISABLE(huart->hdmarx);
						DMA_Set_Config(huart->hdmarx, (uint32_t)&(huart->Instance->RDR), (uint32_t)(rx_buffer), buffer_size);
						__HAL_DMA_ENABLE(huart->hdmarx);
				}

        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RE);
        msu->rx_busy = true;
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
				
        if (rx_idle_callback)
        {
            msu->rx_idle_callback = rx_idle_callback;
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        }
        
    }
    else if (rx)
        result |= MSTR_RX_FAILED;

    return ((MSUART_Start_Transfer_Result)result);
}

void MSUART_Abort_DMA_Transfer(MSUART *msu, bool clear_count)
{
    UART_HandleTypeDef* huart = msu->huart;
    /* Abort Ongoing Transmission */
    if (!msu->hw_rs485)
    {
        if (msu->sw_rs485_tx2rx)
            msu->sw_rs485_tx2rx(msu);
    }
    if (msu->tx_busy)
    {
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAT));
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TCIE));
        __HAL_UART_SEND_REQ(huart, UART_TXDATA_FLUSH_REQUEST);
		if (clear_count)
        {
            msu->tx_size = 0;
            msu->tx_count = 0;
        }
        msu->tx_busy = false;
    }
    if (msu->rx_busy)
    {
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAR));
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_IDLEIE));
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
		if (clear_count)
        {
            msu->rx_size = 0;
            msu->rx_count = 0;
        }
        msu->rx_busy = false;
    }
}

void MSUART_Disable_Transmit(MSUART *msu)
{
    UART_HandleTypeDef* huart = msu->huart;
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TE));
    if (!msu->hw_rs485)
    {
        if (msu->sw_rs485_tx2rx)
            msu->sw_rs485_tx2rx(msu);
    }
    if (msu->tx_busy)
    {
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAT));
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TCIE));
        __HAL_UART_SEND_REQ(huart, UART_TXDATA_FLUSH_REQUEST);
        msu->tx_busy = false;
    }
}

/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/




