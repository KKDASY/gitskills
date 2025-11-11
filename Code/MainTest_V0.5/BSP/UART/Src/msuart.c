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
    GEN_MSUARTX(4)//在这里完成msuart4的初始化
    GEN_UARTX_IRQHANDLER(4)//在这里完成msuart4中断服务函数的定义
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
    else if(uart == UART4)	request_selection = is_tx ? DMA_REQUEST_UART4_TX : DMA_REQUEST_UART4_RX;
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
    //1.通讯参数配置（UART相关）
    UART_Generate_Instance(huart);
	huart->Init.BaudRate = msu->baudrate;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.OverSampling = msu->baudrate > 5000000 ?  UART_OVERSAMPLING_8 : UART_OVERSAMPLING_16;//判断波特率大于5M，过采样率下调至8倍采样（速度高 精度略低），否则16倍采样（精度高）
	huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;//启用1位采样（简化高速通信时的判断）
	huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;//时钟分频：1分频（UART时钟 = 外设时钟）
	huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;//初始化“接收溢出禁用”功能
	huart->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;//禁用接收溢出（接收过快时，不丢弃旧数据）
    /* Call HAL UART Init Function */
    //2.模式选择  普通UART用于近距离通信，RS485用于远距离工业通信（需要方向控制）
	if(msu->hw_rs485 == true)
	{   
        //初始化RS485模式：DE极性高电频有效，无延时
        //函数解释：参数1->对应串口  参数2->高极性（高电平发送）  参数3->接收切换到发送的延时时间（单位us）  参数4->发送切换到接收的延时时间（单位us）
		if (HAL_RS485Ex_Init(huart, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
		    Error_Handler();
	}
	else
	{
        //普通UART模式（无方向切换，直接收发）
		if (HAL_UART_Init(huart) != HAL_OK)
			Error_Handler();
	}
    //3.FIFO模式  启动后可以减少中断次数，提高通信效率（尤其是高速通信的时候）
    //函数解释：参数1->对应串口  参数2->满多少字节FIFO触发中断发送/接收（需要平衡实时性和CPU效率）
	if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
		Error_Handler();
	if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
		Error_Handler();
	if (HAL_UARTEx_EnableFifoMode(huart) != HAL_OK)
		Error_Handler();
    /* Disable IRQ & Clear Flags */
    //4.中断控制  初始化时先禁用所有中断-->清除标志-->再启用需要的中断，避免异常触发
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_TXEIE | 
                                    USART_CR1_PEIE | USART_CR1_CMIE | USART_CR1_RTOIE | USART_CR1_EOBIE |
                                    USART_CR1_TXFEIE | USART_CR1_RXFFIE);
    CLEAR_BIT(huart->Instance->CR2, USART_CR2_LBDIE);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE | USART_CR3_CTSIE | USART_CR3_WUFIE | USART_CR3_TXFTIE |  USART_CR3_RXFTIE);

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF | UART_CLEAR_IDLEF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
	/* Enable IRQ */
    //5.使能中断
    uint32_t irqn = 0;//用于存储中断号
    //判断外设对应的中断号
    if(huart->Instance == USART1)	    irqn = USART1_IRQn;
    else if(huart->Instance == USART2)	irqn = USART2_IRQn;
    else if(huart->Instance == USART3)	irqn = USART3_IRQn;
    else if(huart->Instance == UART4)	irqn = UART4_IRQn;
#if defined(STM32H7)
    else if(huart->Instance == UART4)	irqn = UART4_IRQn;
    else if(huart->Instance == UART5)	irqn = UART5_IRQn;
    else if(huart->Instance == USART6)	irqn = USART6_IRQn;
    else if(huart->Instance == UART7)	irqn = UART7_IRQn;
    else if(huart->Instance == UART8)	irqn = UART8_IRQn;
#endif
    //使能中断
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
/**
*该函数主要用于初始化发送DMA通道和接收DMA通道，配置参数，与UART绑定，实现“UART-DMA”联动
*/
static void MSUART_DMA_Init(MSUART *msu)
{
	UART_HandleTypeDef* huart = msu->huart;//获取uart句柄
	DMA_HandleTypeDef* hdma_tx = huart->hdmatx;//获取发送DMA句柄（HAL库结构体，存储DMA配置）
	DMA_HandleTypeDef* hdma_rx = huart->hdmarx;//获取接收DMA句柄
#if defined(STM32H7)
	DMA_Stream_TypeDef* tx_stream = hdma_tx->Instance;
	DMA_Stream_TypeDef* rx_stream = hdma_rx->Instance;
#elif defined(STM32G4)
	DMA_Channel_TypeDef* tx_stream = hdma_tx->Instance;//发送DMA通道
	DMA_Channel_TypeDef* rx_stream = hdma_rx->Instance;//接收DMA通道
#endif
    //空指针检查（避免硬件配置错误）
	if(huart == NULL || hdma_tx == NULL || hdma_rx == NULL || tx_stream == NULL || rx_stream == NULL)
		Error_Handler();

	/* TX DMA Config */
    //配置发送DMA通道（Tx DMA）
	hdma_tx->Instance = tx_stream;//将DMA句柄与具体的DMA流/通道绑定（形式而已）
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;//外设地址不自增
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;//内存地址自增
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;//外设数据按字节对齐
    hdma_tx->Init.Mode = DMA_NORMAL;//正常模式（非连续发送，需手动启动）
#if defined(STM32H7)
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_tx->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx->Init.MemBurst = DMA_MBURST_INC4;
    hdma_tx->Init.PeriphBurst = DMA_PBURST_SINGLE;
#elif defined(STM32G4)
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;//内存数据按字节对齐
#endif	
	hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;//传输方向：内存->外设
	hdma_tx->Init.Request = UART_Get_DMA_Request_Selection(huart->Instance, true);//获取请求信号（true代表发送，false代表接收）

    if(HAL_DMA_Init(hdma_tx) != HAL_OK)//初始化DMA硬件
		Error_Handler();
    
    /* RX DMA Config */
    hdma_rx->Instance = rx_stream;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;//外设地址非自增
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;//内存地址自增
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;//外设按照字节对齐
    hdma_rx->Init.Mode = DMA_NORMAL;//非连续模式（手动）
#if defined(STM32H7)
	hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_rx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx->Init.MemBurst = DMA_MBURST_INC4;
    hdma_rx->Init.PeriphBurst = DMA_PBURST_SINGLE;
#elif defined(STM32G4)
    hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;//内存按字节对齐
#endif
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;//DMA从外设到内存
    hdma_rx->Init.Request = UART_Get_DMA_Request_Selection(huart->Instance, false);//DMA请求信号  false表示接收

    if(HAL_DMA_Init(hdma_rx) != HAL_OK)//DMA初始化
        Error_Handler();

	/* Link DMA Handler to UART Handler */
    __HAL_LINKDMA(huart, hdmatx, *hdma_tx);//将发送DMA绑定到UART句柄hdmatx
	__HAL_LINKDMA(huart, hdmarx, *hdma_rx);//将接收DMA绑定到UART句柄hdmarx
}

/**
* 该函数响应串口硬件触发的中断事件（这里处理两类事件：发送完成中断和空闲接收中断），分别执行相应的逻辑（切换RS485方向、调用用户设定的回调函数）
*/
static void MSUART_IRQ_Handler(MSUART *msu)
{
    UART_HandleTypeDef* huart = msu->huart;//获取uart句柄（同初始化函数）
    //READ_REG()用于读取硬件寄存器的值
    uint32_t isrflags   = READ_REG(huart->Instance->ISR);//ISR寄存器存储串口当前的中断（ISR）状态
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);//CR1存储串口的中断使能配置

    /*  Transmit Complete */
    //1.如果发送完成中断被配置且被触发
    if ((isrflags & USART_ISR_TC) && (cr1its & USART_CR1_TCIE))
    {
        //如果是非硬件RS485（即使用软件RS485或普通UART）
        if (!msu->hw_rs485)
        {
            if (msu->sw_rs485_tx2rx)//如果设置了“发送转接收”的软件回调函数
                msu->sw_rs485_tx2rx(msu);//执行该函数
        }
        //禁用DMA、发送完成中断并清除标志位
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);//禁用DMA发送（发送完成后关闭DMA通道）
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);//禁用“发送完成中断”（避免重复触发）
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);//清除“发送完成”标志位
        //更新状态：发送不再忙碌（允许下一次发送）
        msu->tx_busy = false;
        //如果串口使能
        if(msu->enable == true)
        {
            msu->tx_count = __HAL_DMA_GET_COUNTER(huart->hdmatx);//获取DMA发送剩余字节数
            if (msu->tx_cplt_callback)//如果用户设置了发送完成回调函数
                msu->tx_cplt_callback(msu);//执行发送完成回调函数
        }
    }
    /* Receive Idle */
    //2.如果空闲（接收）中断被配置且被触发
    else if ((isrflags & USART_ISR_IDLE) && (cr1its & USART_CR1_IDLEIE))
    {
        //关闭DMA接收、空闲接收中断、清除空闲中断标志位
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        //更新状态：接收不再忙碌（允许下次接收）
        msu->rx_busy = false;
        //如果串口使能
        if(msu->enable == true)
        {
            msu->rx_count = __HAL_DMA_GET_COUNTER(huart->hdmarx);//获取DMA接收剩余缓冲区大小
            if (msu->rx_idle_callback)//如果用户设置了“接收空闲回调函数”
                msu->rx_idle_callback(msu);//调用接收空闲回调函数
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
	MSUART_Commu_Init(&msuart4);//初始化msuart配置
	MSUART_DMA_Init(&msuart4);//配置DMA接收和发送
    DISABLE_UARTX_TX_DMA_IRQ(4);//关闭DMA发送中断
    DISABLE_UARTX_RX_DMA_IRQ(4);//关闭DMA接收中断
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

uint32_t tx_buzy = 0;
uint32_t last_tx_buzy = 0;
/**
* 根据参数 tx 和 rx 的值，选择性启动 DMA发送 或 DMA接收，并处理发送/接收过程中的状态（如忙碌判断、强制中断、回调函数注册）。
* @msu                 串口句柄
* @tx                  是否启动发送（true = 启动）
* @force_tx            强制发送（true = 即使正在发送也中断并重启）
* @tx_cplt_callback    发送完成回调函数（发送结束后自动调用）
* @tx_buffer           发送数据缓冲区（要发送的字节数组）
* @tx_size             发送数据长度（字节数）
* @rx                  是否启动接收（true = 启动）
* @force_rx            强制接收（true = 即使正在接收也中断并重启）
* @continue_rx         继续接收（true = 在上次接收基础上继续存数据，false = 新接收覆盖旧数据）
* @rx_idle_callback    接收空闲回调函数
* @rx_buffer           接收数据缓冲区（存放接收字节的数组）
* @buffer_size         接收缓冲区大小（最大可接收字节数）
//返回值               发送/接收是否成功的状态码
*/
MSUART_Start_Transfer_Result MSUART_Start_DMA_Transfer(MSUART *msu, bool tx, bool force_tx, void (*tx_cplt_callback)(MSUART*), const uint8_t *tx_buffer, const uint16_t tx_size, 
                                                                    bool rx, bool force_rx, bool continue_rx, void (*rx_idle_callback)(MSUART*), uint8_t *rx_buffer, const uint16_t buffer_size)
{
    //1.初始化及参数检查
    UART_HandleTypeDef* huart = msu->huart;
    uint8_t result = MSTR_ALL_OK;

    if (msu->enable == false)//如果串口未使能
        return MSTR_NOT_ENABLE;//返回“未使能”错误
    
    if ((tx && rx) && !msu->hw_rs485 && tx_cplt_callback)  //检查“发送接收同时启用的冲突”，可能tx回调函数没有相应处理// if tx and rx are both enabled, rs485 sw flow control must be switched in tx_cplt_callback manually
        return MSTR_PARA_ERROR;

    /* Update TX/RX Status */
    //2.处理当前发送/接收状态（或中断正在进行的传输）
    uint32_t isrflags = READ_REG(huart->Instance->ISR);//读取UART中断状态寄存器（判断是否有未完成的发送）
    //如果当前正在发送
    if (msu->tx_busy)
    {
				if(tx)
				{
					last_tx_buzy = tx_buzy;
					tx_buzy++;
				}
        /* Transmit Complete */
        //情况1：发送已完成（ISR的“发送完成”标志置位），但状态未更新
        if ((isrflags & USART_ISR_TC) && (isrflags & USART_ISR_TXE_TXFNF))
        {
            //手动清除发送相关配置（禁用DMA、中断，清除标志，更新状态）
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);//禁用发送DMA
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);//禁用发送完成中断
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);//清除发送完成标志
            msu->tx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmatx);//获取未传输字节数
            msu->tx_busy = false;//更新状态：发送空闲
        }
        /* Abort Ongoing Transmission */
        //情况2：强制发送（force_tx = true），中断当前发送
        else if (tx && force_tx)
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAT));//禁用发送DMA
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TCIE));//禁用发送完成中断
            __HAL_UART_SEND_REQ(huart, UART_TXDATA_FLUSH_REQUEST);//刷新发送缓冲区（丢弃未发送数据）
            msu->tx_busy = false;//更新状态：发送空闲
        }
    }
    //如果当前正在接收
    if (msu->rx_busy)
    {
        /* Receive Idle */
        //情况1：接收已空闲（ISR的“接收空闲”标志置位），但状态未更新
        if ((isrflags & USART_ISR_IDLE) && !(isrflags & USART_ISR_BUSY))
        {
            //手动清除接收相关配置（禁用DMA、中断，清除标志，更新状态）
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);//禁用接收DMA
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);//禁用接收空闲中断
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);//清除接收空闲标志
            msu->rx_count = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
            msu->rx_busy = false;//更新状态：接收空闲
        }
        /* Abort Ongoing Reception */
        //情况2：强制接收（force_rx = true），中断当前接收
        else if (rx && force_rx)
        {
            ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_DMAR));//禁用接收DMA
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_IDLEIE));//禁用接收空闲中断
            __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);//刷新接收缓冲区（丢弃未读取数据）
            msu->rx_busy = false;//更新状态：接收空闲
        }
    }
    
    /* Start Transmit */
    //3.启动DMA发送
    if (tx && !msu->tx_busy)//如果需要发送且当前发送空闲（未在发送）
    {
		// 清除UART发送相关标志
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        msu->tx_size = tx_size; // 记录发送数据长度
        msu->tx_count = tx_size;// 记录发送剩余字节数（初始=总长度）
        
        //配置发送DMA
        __HAL_DMA_DISABLE(huart->hdmatx);//先禁用DMA
        DMA_Set_Config(huart->hdmatx, (uint32_t)(tx_buffer), (uint32_t)&(huart->Instance->TDR), tx_size);//配置传输参数
        __HAL_DMA_ENABLE(huart->hdmatx);//启用DMA（开始等待UART触发传输）
        ATOMIC_SET_BIT(huart->Instance->CR1, (USART_CR1_TE));//确保UART发送使能
        msu->tx_busy = true;//更新状态：发送中
        
        //如果不是硬件RS485
        if (!msu->hw_rs485)
        {
            //是软件RS485
            if (msu->sw_rs485_rx2tx)
                msu->sw_rs485_rx2tx(msu);// 调用回调函数切换DE/RE引脚为发送模式（高电平）
        }
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);// 启用UART-DMA发送触发（DMA开始传输数据）
         // 注册发送完成回调函数
        if (tx_cplt_callback)
        {
            msu->tx_cplt_callback = tx_cplt_callback;// 保存回调函数指针
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TCIE); // 启用发送完成中断（传输结束后触发回调）
        }
    }
    else if (tx)//如果需要发送但发送忙（未中断且force_tx = false）
        result |= MSTR_TX_FAILED;//发送失败，在结果中标记

    /* Start Receive */
    //4.启动DMA接收
    if (rx && !msu->rx_busy) // 如果需要接收且当前接收空闲（未在接收）
    {
        //清除接收相关标志
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        
        //如果是“非继续接收”模式（可覆盖）
        if (!continue_rx)
        {
			/* Start New Receive */
			msu->rx_size = buffer_size;//记录接收缓冲区大小
			msu->rx_count = buffer_size;//记录接收剩余字节数（初始= 缓冲区大小）
			uint32_t dummy = huart->Instance->RDR;//读取接收数据寄存器（清除旧数据）
			(void)dummy;//避免编译器警告（dummy变量未使用）
            //配置接收DMA
			__HAL_DMA_DISABLE(huart->hdmarx);//禁用DMA接收
			DMA_Set_Config(huart->hdmarx, (uint32_t)&(huart->Instance->RDR), (uint32_t)(rx_buffer), buffer_size);//配置DMA参数
			__HAL_DMA_ENABLE(huart->hdmarx);//开启DMA接收
		}

        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RE); // 确保UART接收使能
        msu->rx_busy = true;//更新状态：接收中
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);// 启用UART-DMA接收触发（DMA开始等待接收数据）
        
        //注册接收空闲回调函数（如果用户提供了回调）
        if (rx_idle_callback)
        {
            msu->rx_idle_callback = rx_idle_callback;// 保存回调函数指针
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);// 启用接收空闲中断（接收结束后触发回调）
        }
        
    }
    else if (rx)//如果需要接收但接收忙（未中断且force_rx = false）
        result |= MSTR_RX_FAILED;//接收失败，在结果中标记

    return (MSUART_Start_Transfer_Result)result;
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
