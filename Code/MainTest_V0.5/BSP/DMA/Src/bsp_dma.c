#include "bsp_dma.h"

//uint32_t DMA_Get_Request_Selection_SPI(SPI_TypeDef* spi, bool is_tx)
//{
//	uint32_t request_selection = 0;
//#if defined(SPI1)
//	if(spi == SPI1)	request_selection = is_tx ? DMA_REQUEST_SPI1_TX : DMA_REQUEST_SPI1_RX;
//#endif
//#if defined(SPI2)
//	else if(spi == SPI2)	request_selection = is_tx ? DMA_REQUEST_SPI2_TX : DMA_REQUEST_SPI2_RX;
//#endif
//#if defined(SPI3)
//	else if(spi == SPI3)	request_selection = is_tx ? DMA_REQUEST_SPI3_TX : DMA_REQUEST_SPI3_RX;
//#endif
//#if defined(SPI4)
//	else if(spi == SPI4)	request_selection = is_tx ? DMA_REQUEST_SPI4_TX : DMA_REQUEST_SPI4_RX;
//#endif
//#if defined(SPI5)
//	else if(spi == SPI5)	request_selection = is_tx ? DMA_REQUEST_SPI5_TX : DMA_REQUEST_SPI5_RX;
//#endif
//#if defined(SPI6)
//	else if(spi == SPI6)	request_selection = is_tx ? BDMA_REQUEST_SPI6_TX : BDMA_REQUEST_SPI6_RX;
//#endif
//	return request_selection;
//}

void DMA_Set_Config(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;
    if(hdma->DMAmuxRequestGen != 0U)
    {
        hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex * 0x1FU));
    hdma->Instance->CNDTR = DataLength;
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
        hdma->Instance->CPAR = DstAddress;
        hdma->Instance->CMAR = SrcAddress;
    }
    else
    {
        hdma->Instance->CPAR = SrcAddress;
        hdma->Instance->CMAR = DstAddress;
    }
}

uint32_t DMA_Get_Request_Selection_I2C(I2C_TypeDef *i2c, bool is_tx)
{
    uint32_t request_selection = 0;
#if defined(I2C1)
    if(i2c == I2C1) request_selection = is_tx ? DMA_REQUEST_I2C1_TX : DMA_REQUEST_I2C1_RX;
#endif
#if defined(I2C2)
    else if(i2c == I2C2) request_selection = is_tx ? DMA_REQUEST_I2C2_TX : DMA_REQUEST_I2C2_RX;
#endif
#if defined(I2C3)
    else if(i2c == I2C3) request_selection = is_tx ? DMA_REQUEST_I2C3_TX : DMA_REQUEST_I2C3_RX;
#endif
#if defined(I2C4)
    else if(i2c == I2C4) request_selection = is_tx ? DMA_REQUEST_I2C4_TX : DMA_REQUEST_I2C4_RX;
#endif
    return request_selection;
}

