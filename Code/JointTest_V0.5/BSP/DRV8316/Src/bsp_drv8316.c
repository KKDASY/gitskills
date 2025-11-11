#include "bsp_drv8316.h"
#include "bsp_dma.h"

#define SPI_READ_REG (uint16_t)(1 << 15)
#define SPI_WRITE_REG (uint16_t)(0 << 15)

#define SPI_SELECT()            DRV8316_SPI_CS_GPIO_PORT->BRR = (uint32_t)DRV8316_SPI_CS_GPIO_PIN
#define SPI_DESELECT()          DRV8316_SPI_CS_GPIO_PORT->BSRR = (uint32_t)DRV8316_SPI_CS_GPIO_PIN
#define SPI_RX_DMA_COMPLETE()   void DRV8316_SPI_DMA_RX_IRQHANDLER(void)
#define SPI_ERROR_CHECK_REG     DRV8316_REG_CR2 

uint32_t DRV_8316_SPI_Error_CNT = 0;
static const DRV8316_CR1_u unlock_reg_value = {.bit_value.REG_LOCK = 0x03};
static const DRV8316_CR1_u lock_reg_value = {.bit_value.REG_LOCK = 0x06};
DRV8316_Reg_t DRV8316_Registers;
uint32_t DRV8316_SPI_Error_CNT;

static uint16_t tx_dma_buffer[2];
static uint16_t rx_dma_buffer[2];
static bool spi_busy;
static uint8_t read_index;

static const DRV8316_Reg_e config_reg_list[CONFIG_REG_NUM] = 
{
    DRV8316_REG_CR2, 
    DRV8316_REG_CR3, 
    DRV8316_REG_CR4, 
    DRV8316_REG_CR5, 
    DRV8316_REG_CR6, 
    DRV8316_REG_CR10
};

static const DRV8316_Reg_e read_reg_list[READ_REG_NUM] = 
{
    DRV8316_REG_SR1, 
    DRV8316_REG_SR2, 
    DRV8316_REG_CR2
};  // DRV8316_REG_CONTROL2 for heartbeat check

static uint8_t* const reg_map[DRV8316_REG_CR10 + 1] =
{
    [DRV8316_REG_ICSR] = (uint8_t*)&DRV8316_Registers.ICSR,
    [DRV8316_REG_SR1] = (uint8_t*)&DRV8316_Registers.SR1,
    [DRV8316_REG_SR2] = (uint8_t*)&DRV8316_Registers.SR2,
    [DRV8316_REG_CR2] = (uint8_t*)&DRV8316_Registers.CR2,
    [DRV8316_REG_CR3] = (uint8_t*)&DRV8316_Registers.CR3,
    [DRV8316_REG_CR4] = (uint8_t*)&DRV8316_Registers.CR4,
    [DRV8316_REG_CR5] = (uint8_t*)&DRV8316_Registers.CR5,
    [DRV8316_REG_CR6] = (uint8_t*)&DRV8316_Registers.CR6,
    [DRV8316_REG_CR10] = (uint8_t*)&DRV8316_Registers.CR10
};

static const DRV8316_RegConfig_t reg_config = 
{
    .CR2.bit_value = {      .CLR_FLT = 1 ,
														.PWM_MODE = CONFIG_PWM_MODE, 
                            .SLEW = CONFIG_SLEW_RATE, 
                            .SDO_MODE = CONFIG_SPI_SDO_MODE,
                            .RSVD = 0x01
                    },
    .CR3.bit_value = {      .OTW_REP = CONFIG_OTP_ON_NFAULT_PIN, 
                            .SPI_FLT_REP = CONFIG_SPIERR_ON_NFAULT_PIN, 
                            .OVP_EN = CONFIG_OVP_ENABLE, 
                            .OVP_SEL = CONFIG_OVP_LEVEL
                    },
    .CR4.bit_value = {      .OCP_MODE = CONFIG_OCP_MODE,
                            .OCP_LVL = CONFIG_OCP_LEVEL,
                            .OCP_DEG = CONFIG_OCP_DEGLITCH
                    },
    .CR5.bit_value = {      .CSA_GAIN = CONFIG_CSA_GAIN},
    .CR6.reg_value = CONFIG_REG_H08_VALUE,
    .CR10.bit_value = {     .DLY_TARGET = CONFIG_DLY_TARGET,
                            .DLYCMP_EN = CONFIG_DLYCMP_ENABLE}
};
/*
* 奇校验
* 返回值：1表示个数为奇数，0表示个数为偶数
*/
static inline uint8_t calculate_parity(uint16_t data)
{
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;//最后一位为1，说明1的个数为奇数，为0说明1的个数为偶数
}

static bool block_write_reg(uint8_t reg_addr, uint8_t write_value, uint16_t* read_data)
{
    bool ok = true;
    uint16_t tx_data = (SPI_WRITE_REG | (reg_addr << 9) | write_value);
    tx_data |= (calculate_parity(tx_data) << 8);

    SPI_SELECT();
    if (HAL_SPI_TransmitReceive(&DRV8316_SPI_HANDLER, (const uint8_t*)&tx_data, (uint8_t*)read_data, 1, 10))
        ok = false;
    SPI_DESELECT();

    return ok;
}

static bool block_read_reg(uint8_t reg_addr, uint16_t* read_data)
{
    bool ok = true;
    uint16_t tx_data = (SPI_READ_REG | (reg_addr << 9));
    tx_data |= (calculate_parity(tx_data) << 8);

    SPI_SELECT();
    if (HAL_SPI_TransmitReceive(&DRV8316_SPI_HANDLER, (const uint8_t*)&tx_data, (uint8_t*)read_data, 1, 10))
        ok = false;
    SPI_DESELECT();

    return ok;
}

static bool block_unlock_reg(void)
{
    uint16_t rx_data = 0;
    return block_write_reg(DRV8316_REG_CR1, unlock_reg_value.reg_value, &rx_data);//向寄存器CR1中写入0x03，解锁寄存器
}

static bool block_lock_reg(void)
{
    uint16_t rx_data = 0;
    return block_write_reg(DRV8316_REG_CR1, lock_reg_value.reg_value, &rx_data);//向寄存器CR1中写入0x06，锁定寄存器
}

void DRV8316_Init(void)
{
    SPI_HandleTypeDef *hspi = &DRV8316_SPI_HANDLER;
    hspi->Instance = DRV8316_SPI_INSTANCE;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = DRV8316_SPI_BAUDRATE_PRESCALER;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 7;
    hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if(HAL_SPI_Init(hspi) != HAL_OK)
    {
        Error_Handler();
    }

    DMA_HandleTypeDef *hdma_tx = hspi->hdmatx;
    DMA_HandleTypeDef *hdma_rx = hspi->hdmarx;

    DMA_Channel_TypeDef *tx_stream = hdma_tx->Instance;
    DMA_Channel_TypeDef *rx_stream = hdma_rx->Instance;

    if(hdma_tx == NULL || hdma_rx == NULL || tx_stream == NULL || rx_stream == NULL)
    {
        Error_Handler();
    }

    hdma_tx->Instance = tx_stream;
    hdma_tx->Init.Request = DMA_Get_Request_Selection_SPI(hspi->Instance, true);
    hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx->Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx->Init.Mode = DMA_NORMAL;
    hdma_tx->Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(hdma_tx) != HAL_OK)
    {
        Error_Handler();
    }

    hdma_rx->Instance = rx_stream;
    hdma_rx->Init.Request = DMA_Get_Request_Selection_SPI(hspi->Instance, false);
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_rx->Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_rx->Init.Mode = DMA_NORMAL;
    hdma_rx->Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(hdma_rx) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, *hdma_tx);
    __HAL_LINKDMA(hspi, hdmarx, *hdma_rx);

    HAL_NVIC_DisableIRQ(DRV8316_SPI_DMA_TX_IRQn);
    HAL_NVIC_DisableIRQ(DRV8316_SPI_DMA_RX_IRQn);

    DRV_8316_SPI_Error_CNT = 1;
}
/*
* DRV8316电机驱动芯片的寄存器配置函数
* 返回值：true-配置成功   false - 配置失败
*/
bool DRV8316_Config_Registers(void)
{
    uint16_t rx_data = 0;//用于存储从SPI接收的数据
    DRV8316_RegConfig_t actual_config = {0};//存储实际读取的配置值
    
    //1.解锁寄存器（某些寄存器需要先解锁才能写入）
    if(!block_unlock_reg())
    {
        return false;//解锁失败则直接返回错误
    }
    //2.遍历所有需要配置的寄存器（）
    for(uint8_t i = 0; i < CONFIG_REG_NUM; i++)
    {
        //写入配置值到当前寄存器
        if(!block_write_reg(config_reg_list[i], ((uint8_t*)&reg_config)[i], &rx_data))
        {
            return false;
        }
        //保存ICSR寄存器的值（高8位）
        DRV8316_Registers.ICSR.reg_value = rx_data >> 8;
        //保存实际写入的值（低8位）
        ((uint8_t*)&actual_config)[i] = rx_data & 0xFF;
    }
    //3.重新锁定寄存器（写保护）
    if(!block_lock_reg())
    {
        return false;
    }
    //4.更新本地寄存器结构体的关键字段
    DRV8316_Registers.CR2.reg_value = actual_config.CR2.reg_value;
    DRV8316_Registers.CR3.reg_value = actual_config.CR3.reg_value;
    DRV8316_Registers.CR4.reg_value = actual_config.CR4.reg_value;
    DRV8316_Registers.CR5.reg_value = actual_config.CR5.reg_value;
    DRV8316_Registers.CR6.reg_value = actual_config.CR6.reg_value;
    DRV8316_Registers.CR10.reg_value = actual_config.CR10.reg_value;
    
    //5.验证阶段：重新读取所有寄存器确认配置正确
    for(uint8_t i = 0;i < CONFIG_REG_NUM;i++)
    {
        if(!block_read_reg(config_reg_list[i], &rx_data))//读取寄存器
        {
            return false;//读取失败则返回错误
        }
        //更新ICSR和实际配置值
        DRV8316_Registers.ICSR.reg_value = rx_data >> 8;
        ((uint8_t*)&actual_config)[i] = rx_data & 0xFF;
        
        //比较读取值与期望值
        if(((uint8_t*)&actual_config)[i] != ((uint8_t*)&reg_config)[i])
        {
            return false;
        }
    }
    //6.再次更新本地寄存器结构体（保证数据一致）
    DRV8316_Registers.CR2.reg_value = actual_config.CR2.reg_value;
    DRV8316_Registers.CR3.reg_value = actual_config.CR3.reg_value;
    DRV8316_Registers.CR4.reg_value = actual_config.CR4.reg_value;
    DRV8316_Registers.CR5.reg_value = actual_config.CR5.reg_value;
    DRV8316_Registers.CR6.reg_value = actual_config.CR6.reg_value;
    DRV8316_Registers.CR10.reg_value = actual_config.CR10.reg_value;
    
    //7.清零SPI错误计数器并返回成功
    DRV8316_SPI_Error_CNT = 0;
    return true;
}

static inline void spi_clear_error_flags(SPI_HandleTypeDef *hspi)
{
    __IO uint32_t tmpreg;

    tmpreg = hspi->Instance->SR;
    tmpreg = hspi->Instance->DR;

    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
}

static void drv8316_read_write_dma(void)
{
    SPI_HandleTypeDef *hspi = &DRV8316_SPI_HANDLER;

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
    DMA_Set_Config(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)rx_dma_buffer, 1);
    __HAL_DMA_ENABLE_IT(hspi->hdmarx, DMA_IT_TC);
    __HAL_DMA_ENABLE(hspi->hdmarx);
    /* Enable Rx DMA Request */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

    /* Enable the Tx DMA Stream/Channel  */
    __HAL_DMA_DISABLE(hspi->hdmatx);
    DMA_Set_Config(hspi->hdmatx, (uint32_t)tx_dma_buffer, (uint32_t)&hspi->Instance->DR, 1);
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


SPI_RX_DMA_COMPLETE()
{
   DMA_HandleTypeDef *hdma = DRV8316_SPI_HANDLER.hdmarx;
   uint32_t flag_it = hdma->DmaBaseAddress->ISR;

   // /* Transfer Complete Interrupt management ***********************************/
   if (flag_it & ((uint32_t)DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1FU)))
   {
       __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
       /* Clear the transfer complete flag */
       hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_TCIF1 << (hdma->ChannelIndex & 0x1FU));
       SPI_DESELECT();
       spi_busy = false;

       DRV8316_Registers.ICSR.reg_value = rx_dma_buffer[0] >> 8;
       uint8_t* const reg_ptr = reg_map[read_reg_list[read_index]];
       if (reg_ptr)
       {
            if (read_reg_list[read_index] == SPI_ERROR_CHECK_REG)
            {
                if ((rx_dma_buffer[0] & 0xFF) != *reg_ptr)
                {
                    DRV8316_SPI_Error_CNT++;
                }
            }
            else
            {
                *reg_ptr = rx_dma_buffer[0] & 0xFF;
            }
       }
       read_index++;
       if (read_index < READ_REG_NUM)
       {
            tx_dma_buffer[0] = (SPI_READ_REG | (read_reg_list[read_index] << 9));
            drv8316_read_write_dma();
       }
   }
}

void DRV8316_Update_Reg_DMA(void)
{
    read_index = 0;
    tx_dma_buffer[0] = (SPI_READ_REG | (read_reg_list[read_index] << 9));
    drv8316_read_write_dma();
}

