
/**
*****************************************************************************************
 * @file     kth5701.c
 * @author   Michael Zhou
 * 
 * @brief    
 * 
 * @date     2025-05-07
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

#include "kth5701.h"
#include "bsp_dma.h"

/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/

#define	I2C_ADDRESS                 0xD0

#define CMD_CONTINUOUS_MODE         0x10
#define	CMD_WAKEUP_SLEEP_MODE       0x20
#define	CMD_SINGLE_MODE             0x30
#define	CMD_READ_MEASUREMENT        0x40
#define	CMD_READ_REGISTER           0x50
#define	CMD_WRITE_REGISTER          0x60
#define	CMD_IDLE_MODE               0x80
#define	CMD_RESET                   0xF0

/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/



/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

typedef enum
{
    ASYNC_READ_REGISTER,
    ASYNC_WRITE_REGISTER,
    ASYNC_READ_MEASUREMENT

} I2C_Async_Type_E;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

typedef struct
{
    I2C_Async_Type_E                async_type;

    uint8_t                         reg_addr;
    uint16_t                        reg_value;

    void                            (*read_measurement_cplt_cb)(void);
    void                            (*read_reg_cplt_cb)(uint16_t read_value);
    void                            (*write_reg_cplt_cb)(void);

} I2C_Async_Ctrl_T;

typedef struct
{
    KTH5701_Reg_H1C_U              reg_H1C;
    KTH5701_Reg_H1D_U              reg_H1D;
    KTH5701_Reg_H1E_U              reg_H1E;

} KTH5701_Reg_Config_T;


/**************************************************************************************************
---- Constants ----------------------------------------------------------------------------------*/

static uint16_t* const reg_map[KTH5701_REG_H1F + 1] = 
{
    [KTH5701_REG_H06] = &KTH5701_Registers.reg_H06.reg_value,
    [KTH5701_REG_H0D] = &KTH5701_Registers.reg_H0D.CHIP_ID,
    [KTH5701_REG_H14] = &KTH5701_Registers.reg_H14.OFFSET_X,
    [KTH5701_REG_H15] = &KTH5701_Registers.reg_H15.OFFSET_Y,
    [KTH5701_REG_H16] = &KTH5701_Registers.reg_H16.OFFSET_Z,
    [KTH5701_REG_H17] = &KTH5701_Registers.reg_H17.reg_value,
    [KTH5701_REG_H18] = &KTH5701_Registers.reg_H18.reg_value,
    [KTH5701_REG_H19] = &KTH5701_Registers.reg_H19.WXY_TH,
    [KTH5701_REG_H1A] = &KTH5701_Registers.reg_H1A.WZ_TH,
    [KTH5701_REG_H1B] = &KTH5701_Registers.reg_H1B.reg_value,
    [KTH5701_REG_H1C] = &KTH5701_Registers.reg_H1C.reg_value,
    [KTH5701_REG_H1D] = &KTH5701_Registers.reg_H1D.reg_value,
    [KTH5701_REG_H1E] = &KTH5701_Registers.reg_H1E.reg_value,
    [KTH5701_REG_H1F] = &KTH5701_Registers.reg_H1F.PUSH_CONFIG,
};

#define CONFIG_REG_NUM 3

static const KTH5701_Reg_Config_T reg_config = 
{
    .reg_H1C.bit_value = { .TEMP_OSR = 0, .MAGN_OSR = 0, .GAIN = 6, .DIG_CTRL = 0 },
    .reg_H1D.reg_value = 0x0000,
    .reg_H1E.bit_value = { .ANG_MAGN_SEL = KTH5701_CONFIG_MEASURE_MODE },
};

static const KTH5701_Reg_E config_reg_list[CONFIG_REG_NUM] = 
{
    KTH5701_REG_H1C,
    KTH5701_REG_H1D,
    KTH5701_REG_H1E
};

/**************************************************************************************************
---- Private Variables --------------------------------------------------------------------------*/

static uint8_t tx_buffer[KTH5701_I2C_BUFFER_SIZE];
static uint8_t rx_buffer[KTH5701_I2C_BUFFER_SIZE];
static I2C_Async_Ctrl_T i2c_async_ctrl;

/**************************************************************************************************
---- Exported Variables -------------------------------------------------------------------------*/

extern I2C_HandleTypeDef KTH5701_I2C_HANDLER;

volatile KTH5701_Mode_E KTH5701_Mode = KTH5701_MODE_UNKNOW;
KTH5701_Reg_T KTH5701_Registers;
KTH5701_Raw_T KTH5701_Raw_Measurement;
uint32_t KTH5701_Error_Counter;

/**************************************************************************************************
---- Private Function Prototypes ----------------------------------------------------------------*/

// 内部函数原型  static void private_func(void);

/**************************************************************************************************
---- Private Function Implementations------------------------------------------------------------*/

static void update_measurement(void)
{
    if (KTH5701_Registers.reg_H1E.bit_value.ANG_MAGN_SEL == KTH5701_MEASURE_TXYZ)
    {
        KTH5701_Raw_Measurement.TXYZ.T = (rx_buffer[1] << 8) | rx_buffer[2];
        KTH5701_Raw_Measurement.TXYZ.X = (rx_buffer[3] << 8) | rx_buffer[4];
        KTH5701_Raw_Measurement.TXYZ.Y = (rx_buffer[5] << 8) | rx_buffer[6];
        KTH5701_Raw_Measurement.TXYZ.Z = (rx_buffer[7] << 8) | rx_buffer[8];
    }
    else
    {
        KTH5701_Raw_Measurement.TABZ.T = (rx_buffer[1] << 8) | rx_buffer[2];
        KTH5701_Raw_Measurement.TABZ.A = (rx_buffer[3] << 8) | rx_buffer[4];
        KTH5701_Raw_Measurement.TABZ.B = (rx_buffer[5] << 8) | rx_buffer[6];
        KTH5701_Raw_Measurement.TABZ.Z = (rx_buffer[7] << 8) | rx_buffer[8];
    }
}

static void status_update(uint16_t status_value)
{
    KTH5701_Registers.reg_H06.reg_value = status_value;
    KTH5701_Reg_H06_U reg_h06 = KTH5701_Registers.reg_H06;
    if (reg_h06.bit_value.CONTINUOUS)
    {
        KTH5701_Mode = KTH5701_MODE_CONTINUOUS;
    }
    else if (reg_h06.bit_value.WAKE_UP)
    {
        KTH5701_Mode = KTH5701_MODE_WAKE_UP_SLEEP;
    }
    else if (reg_h06.bit_value.SINGLE)
    {
        KTH5701_Mode = KTH5701_MODE_SINGLE;
    }
    else
    {
        KTH5701_Mode = KTH5701_MODE_IDLE;
    }
}

static bool block_switch_to_idle_mode(void)
{
    if (KTH5701_Mode == KTH5701_MODE_IDLE)
        return true;
    
    if (HAL_I2C_Mem_Read(&KTH5701_I2C_HANDLER, I2C_ADDRESS, CMD_IDLE_MODE, I2C_MEMADD_SIZE_8BIT, rx_buffer, 1, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    status_update(rx_buffer[0]);
    HAL_Delay(1);
    return true;
}



/**************************************************************************************************
---- Exported Function Implementations-----------------------------------------------------------*/

void KTH5701_Init(void)
{
  //获取KTH5701对应的I2C句柄  
	I2C_HandleTypeDef *hi2c = &KTH5701_I2C_HANDLER;
  //初始化I2C硬件配置
	KTH5701_I2C_INIT();

    DMA_HandleTypeDef* hdma_tx = hi2c->hdmatx;
	DMA_HandleTypeDef* hdma_rx = hi2c->hdmarx;

	DMA_Channel_TypeDef* tx_stream = hdma_tx->Instance;
	DMA_Channel_TypeDef* rx_stream = hdma_rx->Instance;

	if (hdma_tx == NULL || hdma_rx == NULL || tx_stream == NULL || rx_stream == NULL)
		Error_Handler();

	/* TX DMA Config */
	hdma_tx->Instance = tx_stream;
    hdma_tx->Init.Request = DMA_Get_Request_Selection_I2C(hi2c->Instance, true);
    hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx->Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx->Init.Mode = DMA_NORMAL;
    hdma_tx->Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(hdma_tx) != HAL_OK)
		Error_Handler();
    
    /* RX DMA Config */
    hdma_rx->Instance = rx_stream;
    hdma_rx->Init.Request = DMA_Get_Request_Selection_I2C(hi2c->Instance, false);
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx->Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx->Init.Mode = DMA_NORMAL;
    hdma_rx->Init.Priority = DMA_PRIORITY_LOW;

    if(HAL_DMA_Init(hdma_rx) != HAL_OK)
        Error_Handler();

	/* Link DMA Handler to SPI Handler */
    __HAL_LINKDMA(hi2c, hdmatx, *hdma_tx);
	__HAL_LINKDMA(hi2c, hdmarx, *hdma_rx);

    HAL_NVIC_EnableIRQ(KTH5701_I2C_DMA_TX_IRQN);
    HAL_NVIC_EnableIRQ(KTH5701_I2C_DMA_RX_IRQN);
	
	HAL_Delay(5);
	//发送硬件复位命令
	KTH5701_Block_Reset();
	/* Read All Registers */
		//读取所有寄存器（初始化校验）
		//遍历KTH5701的所有寄存器（地址从0到KTH5701_REG_H1F）
    for (uint8_t i = 0; i < KTH5701_REG_H1F + 1; i++)
    {
			//如果寄存器映射表中该地址有效
        if (reg_map[i] != NULL)
					//读取该寄存器的值（读地址i的值，不存）
            KTH5701_Block_Read_Reg(i, NULL);
    }
}

bool KTH5701_Config_Registers(void)
{ 
    /* Write Config Registers */
    for (uint8_t i = 0; i < CONFIG_REG_NUM; i++)
    {
        if (KTH5701_Block_Write_Reg(config_reg_list[i], ((uint16_t*)&reg_config)[i]) == false)
            return false;
    }
    return true;
}

bool KTH5701_Block_Switch_Mode(KTH5701_Mode_E mode)
{
    if (KTH5701_Mode == mode)
        return true;
    /* switch to idle mode */
    if (block_switch_to_idle_mode() == false)
        return false;
    if (mode == KTH5701_MODE_IDLE)
        return true;
    /* switch to continuous mode */
    if (mode == KTH5701_MODE_CONTINUOUS)
    {
        uint8_t cmd = CMD_CONTINUOUS_MODE | 0xF;
        if (HAL_I2C_Mem_Read(&KTH5701_I2C_HANDLER, I2C_ADDRESS, cmd, I2C_MEMADD_SIZE_8BIT, rx_buffer, 1, 10) != HAL_OK)
        {
            KTH5701_Error_Counter++;
            return false;
        }
        status_update(rx_buffer[0]);
        HAL_Delay(1);
        return true;
    }
    /* switch to wakeup & sleep mode */
    if (mode == KTH5701_MODE_WAKE_UP_SLEEP)
    {
        uint8_t cmd = CMD_WAKEUP_SLEEP_MODE | 0xF;
        if (HAL_I2C_Mem_Read(&KTH5701_I2C_HANDLER, I2C_ADDRESS, cmd, I2C_MEMADD_SIZE_8BIT, rx_buffer, 1, 10) != HAL_OK)
        {
            KTH5701_Error_Counter++;
            return false;
        }
        status_update(rx_buffer[0]);
        HAL_Delay(1);
        return true;
    }
    /* switch to single mode */
    if (mode == KTH5701_MODE_SINGLE)
    {
        uint8_t cmd = CMD_SINGLE_MODE | 0xF;
        if (HAL_I2C_Mem_Read(&KTH5701_I2C_HANDLER, I2C_ADDRESS, cmd, I2C_MEMADD_SIZE_8BIT, rx_buffer, 1, 10) != HAL_OK)
        {
            KTH5701_Error_Counter++;
            return false;
        }
        status_update(rx_buffer[0]);  // single mode will automatically switch to idle mode after measurement
        HAL_Delay(1);
        return true;
    }
    return false;
}

bool KTH5701_Block_Reset(void)
{
    /* switch to idle mode */
    if (block_switch_to_idle_mode() == false)
        return false;
    tx_buffer[0] = CMD_RESET;
		//发送复位指令
    if (HAL_I2C_Master_Transmit(&KTH5701_I2C_HANDLER, I2C_ADDRESS, tx_buffer, 1, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    HAL_Delay(5);
		//接收复位结果
	if (HAL_I2C_Master_Receive(&KTH5701_I2C_HANDLER, I2C_ADDRESS, rx_buffer, 1, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
		
	status_update(rx_buffer[0]);//根据接收字节更新传感器状态
    if (KTH5701_Registers.reg_H06.bit_value.SOFT_RST == 0)
			return false;//SOFT_RST为0 表示复位失败
    return true;
}

bool KTH5701_Block_Switch_Measurement_Mode(KTH5701_Measure_Type_E mtype)
{
    /* switch to idle mode */
    if (block_switch_to_idle_mode() == false)
        return false;
    KTH5701_Reg_H1E_U h1e_new_value = { .reg_value = KTH5701_Registers.reg_H1E.reg_value };
    h1e_new_value.bit_value.ANG_MAGN_SEL = (mtype == KTH5701_MEASURE_TXYZ ? 0x0 : 0x1);
    return KTH5701_Block_Write_Reg(KTH5701_REG_H1E, h1e_new_value.reg_value);
}

bool KTH5701_Block_Read_Reg(KTH5701_Reg_E reg_addr, uint16_t *read_value)
{
    tx_buffer[0] = CMD_READ_REGISTER;//读寄存器命令
    tx_buffer[1] = reg_addr << 2;//寄存器地址
    if (HAL_I2C_Master_Transmit(&KTH5701_I2C_HANDLER, I2C_ADDRESS, tx_buffer, 2, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    if (HAL_I2C_Master_Receive(&KTH5701_I2C_HANDLER, I2C_ADDRESS, rx_buffer, 3, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    status_update(rx_buffer[0]);//传感器状态更新
    uint16_t reg_value = (rx_buffer[1] << 8) | rx_buffer[2];//16bit寄存器值，大端传输
    if (reg_map[reg_addr] != NULL && read_value != NULL)
        *read_value = reg_value;
    return true;
}

bool KTH5701_Block_Write_Reg(KTH5701_Reg_E reg_addr, uint16_t write_value)
{
    tx_buffer[0] = CMD_WRITE_REGISTER;
    tx_buffer[1] = write_value >> 8;
    tx_buffer[2] = write_value & 0xFF;
    tx_buffer[3] = reg_addr << 2;
    if (HAL_I2C_Master_Transmit(&KTH5701_I2C_HANDLER, I2C_ADDRESS, tx_buffer, 4, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    if (reg_map[reg_addr] != NULL)
        *reg_map[reg_addr] = write_value;
    if (HAL_I2C_Master_Receive(&KTH5701_I2C_HANDLER, I2C_ADDRESS, rx_buffer, 1, 10) != HAL_OK)
    {
        KTH5701_Error_Counter++;
    }
    else
    {
        status_update(rx_buffer[0]);
    }
    return true;
}

bool KTH5701_Async_Read_Reg(KTH5701_Reg_E reg_addr, void (*read_cplt_cb)(uint16_t read_value))
{
    tx_buffer[0] = CMD_READ_REGISTER;
    tx_buffer[1] = reg_addr << 2;
    if (HAL_I2C_Master_Seq_Transmit_DMA(&KTH5701_I2C_HANDLER, I2C_ADDRESS, tx_buffer, 2, I2C_FIRST_FRAME) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    i2c_async_ctrl.async_type = ASYNC_READ_REGISTER;
    i2c_async_ctrl.reg_addr = reg_addr;
    i2c_async_ctrl.read_reg_cplt_cb = read_cplt_cb;
    return true;
}

bool KTH5701_Async_Write_Reg(KTH5701_Reg_E reg_addr, uint16_t write_value, void (*write_cplt_cb)(void))
{
    tx_buffer[0] = CMD_WRITE_REGISTER;
    tx_buffer[1] = write_value >> 8;
    tx_buffer[2] = write_value & 0xFF;
    tx_buffer[3] = reg_addr << 2;
    if (HAL_I2C_Master_Seq_Transmit_DMA(&KTH5701_I2C_HANDLER, I2C_ADDRESS, tx_buffer, 4, I2C_FIRST_FRAME) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    i2c_async_ctrl.async_type = ASYNC_WRITE_REGISTER;
    i2c_async_ctrl.write_reg_cplt_cb = write_cplt_cb;
    i2c_async_ctrl.reg_addr = reg_addr;
    i2c_async_ctrl.reg_value = write_value;
    return true;
}

bool KTH5701_Async_Read_Measurement(void (*read_cplt_cb)(void))
{
    const uint8_t cmd_read_4measurement = CMD_READ_MEASUREMENT | 0xF;
    if (HAL_I2C_Mem_Read_DMA(&KTH5701_I2C_HANDLER, I2C_ADDRESS, cmd_read_4measurement, I2C_MEMADD_SIZE_8BIT, rx_buffer, 9) != HAL_OK)
    {
        KTH5701_Error_Counter++;
        return false;
    }
    i2c_async_ctrl.async_type = ASYNC_READ_MEASUREMENT;
    i2c_async_ctrl.read_measurement_cplt_cb = read_cplt_cb;
    return true;
}

void KTH5701_I2C_Mem_Rx_Cplt_Callback_Process(void)
{
    switch ((uint8_t)(i2c_async_ctrl.async_type))
    {
        case ASYNC_READ_MEASUREMENT:
            status_update(rx_buffer[0]);
            update_measurement();
            if (i2c_async_ctrl.read_measurement_cplt_cb != NULL)
                i2c_async_ctrl.read_measurement_cplt_cb();
            break;
        
    }
}

void KTH5701_I2C_Master_Tx_Cplt_Callback_Process(void)
{
    switch ((uint8_t)(i2c_async_ctrl.async_type))
    {
        case ASYNC_READ_REGISTER:
            if (HAL_I2C_Master_Seq_Receive_DMA(&KTH5701_I2C_HANDLER, I2C_ADDRESS, rx_buffer, 3, I2C_LAST_FRAME) != HAL_OK)
                KTH5701_Error_Counter++;
            break;
        case ASYNC_WRITE_REGISTER:
            if (reg_map[i2c_async_ctrl.reg_addr] != NULL)
                *reg_map[i2c_async_ctrl.reg_addr] = i2c_async_ctrl.reg_value;
            if (HAL_I2C_Master_Seq_Receive_DMA(&KTH5701_I2C_HANDLER, I2C_ADDRESS, rx_buffer, 1, I2C_LAST_FRAME) != HAL_OK)
                KTH5701_Error_Counter++;
            break;
    }
}

void KTH5701_I2C_Master_Rx_Cplt_Callback_Process(void)
{
    switch ((uint8_t)(i2c_async_ctrl.async_type))
    {
        case ASYNC_READ_REGISTER:
            status_update(rx_buffer[0]);
            uint16_t reg_value = (rx_buffer[1] << 8) | rx_buffer[2];
            if (reg_map[i2c_async_ctrl.reg_addr] != NULL)
                *reg_map[i2c_async_ctrl.reg_addr] = reg_value;
            if (i2c_async_ctrl.read_reg_cplt_cb != NULL)
                i2c_async_ctrl.read_reg_cplt_cb(reg_value);
            break;
        case ASYNC_WRITE_REGISTER:
            status_update(rx_buffer[0]);
            if (i2c_async_ctrl.write_reg_cplt_cb != NULL)
                i2c_async_ctrl.write_reg_cplt_cb();
            break;
    }
}


/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/
