/**
*****************************************************************************************
 * @file     kth7816.c
 * @author   Michael Zhou
 * 
 * @brief    
 * 
 * @date     2025-05-05
 * @version  0.0.1
 * 
 * @note     
 * @warning  
 * 
 * Copyright (c) WUJITECH 2019-2025. All rights reserved.
*****************************************************************************************
*/
/**************************************************************************************************
---- Includes -----------------------------------------------------------------------------------*/

#include "kth7816.h"
#include "bsp_dma.h"

/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/

#define SPI_READ_REG            (uint16_t)(0x01 << 14)
#define SPI_WRITE_REG           (uint16_t)(0x10 << 14)

#define REG_MAG_WARNING         (uint16_t)((0x06 & 0x3F) << 9)
#define REG_RD                  (uint16_t)((0x09 & 0x3F) << 9)

#define SPI_READ_MAG_CMD        (uint16_t)(SPI_READ_REG | REG_MAG_WARNING)

#define SPI_GET_POS_CMD         (uint16_t)(0x00)
#define SPI_WRDIS_CMD           (uint16_t)(0xE802)

/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/

#define SPI_SELECT()            KTH7816_SPI_CS_GPIO_PORT->BRR = (uint32_t)KTH7816_SPI_CS_GPIO_PIN
#define SPI_DESELECT()          KTH7816_SPI_CS_GPIO_PORT->BSRR = (uint32_t)KTH7816_SPI_CS_GPIO_PIN

#define SPI_RX_DMA_COMPLETE()   void KTH7816_SPI_DMA_RX_IRQHANDLER(void)

/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

// 枚举  typedef enum { OK, FAILED, ERROR } Status;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

typedef union
{
    struct
    {
        uint8_t padding:2;
        uint8_t mgl:3;
        uint8_t mgh:3;

    } bit_member;
    uint8_t value;
    
} Mag_Status_U;

/**************************************************************************************************
---- Constants ----------------------------------------------------------------------------------*/



/**************************************************************************************************
---- Private Variables --------------------------------------------------------------------------*/

static bool is_wrdis = false;     // Disable read/write register function (can only read absolute position), requires power cycling to restore
static bool spi_busy = false;

static uint16_t spi_pos_cmd_buffer[2] = {SPI_GET_POS_CMD};
static uint16_t spi_cmd_buffer[2] = {0};

/**************************************************************************************************
---- Exported Variables -------------------------------------------------------------------------*/

extern SPI_HandleTypeDef KTH7816_SPI_HANDLER;

uint32_t KTH7816_Raw_Pos;

/**************************************************************************************************
---- Private Function Prototypes ----------------------------------------------------------------*/



/**************************************************************************************************
---- Private Function Implementations------------------------------------------------------------*/


/**************************************************************************************************
---- Exported Function Implementations-----------------------------------------------------------*/

SPI_RX_DMA_COMPLETE()
{
   DMA_HandleTypeDef *hdma = KTH7816_SPI_HANDLER.hdmarx;
   uint32_t flag_it = hdma->DmaBaseAddress->ISR;

   // /* Transfer Complete Interrupt management ***********************************/
   if (flag_it & ((uint32_t)DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1FU)))
   {
       __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
       /* Clear the transfer complete flag */
       hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_TCIF1 << (hdma->ChannelIndex & 0x1FU));
       SPI_DESELECT();
       spi_busy = false;
   }
}

static inline void spi_clear_error_flags(SPI_HandleTypeDef *hspi)
{
    __IO uint32_t tmpreg;

    tmpreg = hspi->Instance->SR;
    tmpreg = hspi->Instance->DR;

    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
}

void KTH7816_Init(void)
{
    SPI_HandleTypeDef *hspi = &KTH7816_SPI_HANDLER;
    hspi->Instance = KTH7816_SPI_INSTANCE;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = KTH7816_SPI_BAUDRATE_PRESCALER;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 7;
    hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(hspi) != HAL_OK)
    {
        Error_Handler();
    }

	DMA_HandleTypeDef* hdma_tx = hspi->hdmatx;
	DMA_HandleTypeDef* hdma_rx = hspi->hdmarx;

	DMA_Channel_TypeDef* tx_stream = hdma_tx->Instance;
	DMA_Channel_TypeDef* rx_stream = hdma_rx->Instance;

	if (hdma_tx == NULL || hdma_rx == NULL || tx_stream == NULL || rx_stream == NULL)
		Error_Handler();

	/* TX DMA Config */
	hdma_tx->Instance = tx_stream;
    hdma_tx->Init.Request = DMA_Get_Request_Selection_SPI(hspi->Instance, true);
    hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tx->Init.Mode = DMA_NORMAL;
    hdma_tx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if(HAL_DMA_Init(hdma_tx) != HAL_OK)
		Error_Handler();
    
    /* RX DMA Config */
    hdma_rx->Instance = rx_stream;
    hdma_rx->Init.Request = DMA_Get_Request_Selection_SPI(hspi->Instance, false);
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_rx->Init.Mode = DMA_NORMAL;
    hdma_rx->Init.Priority = DMA_PRIORITY_VERY_HIGH;

    if(HAL_DMA_Init(hdma_rx) != HAL_OK)
        Error_Handler();

	/* Link DMA Handler to SPI Handler */
    __HAL_LINKDMA(hspi, hdmatx, *hdma_tx);
	__HAL_LINKDMA(hspi, hdmarx, *hdma_rx);

    HAL_NVIC_DisableIRQ(KTH7816_SPI_DMA_TX_IRQN);
    HAL_NVIC_EnableIRQ(KTH7816_SPI_DMA_RX_IRQN);
}

void KTH7816_Disable_Read_Write(void)
{
    spi_cmd_buffer[0] = SPI_WRDIS_CMD;
    HAL_SPI_Transmit(&KTH7816_SPI_HANDLER, (const uint8_t *)&spi_cmd_buffer, 1, 10);
    is_wrdis = true;
}

void KTH7816_Start_Read_Pos_DMA(void)
{
    SPI_HandleTypeDef *hspi = &KTH7816_SPI_HANDLER;

    if (spi_busy)
    {
        SPI_DESELECT();
        __HAL_SPI_DISABLE(hspi);
    }

    spi_busy = true;
    SPI_SELECT();

    spi_clear_error_flags(hspi);

    /* Reset the threshold bit */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_LDMATX | SPI_CR2_LDMARX);
    /* Set fiforxthreshold according the reception data length: 16bit */
    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);

    /* Enable the Rx DMA Stream/Channel  */
    __HAL_DMA_DISABLE(hspi->hdmarx);
    DMA_Set_Config(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)&KTH7816_Raw_Pos, 1);
    __HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC);
    __HAL_DMA_ENABLE(hspi->hdmarx);
    /* Enable Rx DMA Request */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

    /* Enable the Tx DMA Stream/Channel  */
    __HAL_DMA_DISABLE(hspi->hdmatx);
    DMA_Set_Config(hspi->hdmatx, (uint32_t)spi_pos_cmd_buffer, (uint32_t)&hspi->Instance->DR, 1);
    __HAL_DMA_ENABLE(hspi->hdmatx);
    /* Check if the SPI is already enabled */
    if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }
    /* Enable Tx DMA Request */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
}

/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/
