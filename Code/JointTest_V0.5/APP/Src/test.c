#include "test.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "adc.h"
#include "kth5701.h"
#include "bsp_tim.h"
#include "bsp_drv8316.h"
#include "msuart.h"
#include "Motor.h"

extern TestItem_t test_item;
extern tAnalogValue Sys_AnalogTemp;
extern uint16_t KTH7816_Raw_Pos;
extern DRV8316_Reg_t DRV8316_Registers;
extern UART_Buffers uart_buffers;
extern Timeout_t timeout_manager;
extern bool test_item_valid;
extern Drive_t drive_config;
extern float MAXValue;
extern uint32_t DRV8316_SPI_Error_CNT;

TestItem_t result_feedback = {.raw_value = 0};

void Test()
{   
    if(test_item_valid)
    {
        if(test_item.bits.Item_BusVoltSample)
        {
            BusVoltSample_MainProcess();
        }
        if(test_item.bits.Item_RS485TxRx)//待接收RS485信息
        {
            RS485TxRx_MainProcess();//RS485接收
        }
        if(test_item.bits.Item_InputEncoderSPI)
        {
            InputEncoderSPI_MainProcess();
        }
        if(test_item.bits.Item_OutputEncoderI2C)
        {
            OutputEncoderI2C_MainProcess();
        }
        if(test_item.bits.Item_VrefSample)
        {
            VrefSample_MainProcess();
        }
        if(test_item.bits.Item_PhaseCurrentSample)
        {
            PhaseCurrentSample_MainProcess();
        }
        if(test_item.bits.Item_DRV8316State)
        {
            DRV8316State_MainProcess();
        }
        if(test_item.bits.Item_DRV8316Drive)
        {
            DRV8316Drive_MainProcess();
        }
        if(test_item.bits.Item_FDCAN)
        {
            //Reserved
        }
        if(test_item.bits.Item_USB)
        {
            //Reserved
        }
        if(test_item.bits.Item_PHYVoltSample)
        {
            //Reserved
        }
        if(test_item.bits.Item_IMUSPI)
        {
            //Reserved
        }
        while(timeout_manager.waiting)
        {
            SystemTimer_Delay_ms(1);
        }
        uint16_t len = Pack(ID_Main, (uint8_t *)uart_buffers.tx_dma_buffer, true, test_item, result_feedback);
        MSUART_Start_DMA_Transfer(&msuart3, true, false, NULL, (const uint8_t *)uart_buffers.tx_dma_buffer, len, true, true, false, UART_RxCallback, (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
        SystemTimer_Delay_ms(10);
        result_feedback.raw_value = 0;
        test_item_valid = false;
    }
    else
    {
				// 如果没有收到数据翻转LED
//				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
        SystemTimer_Delay_ms(1);
    }
}

void BusVoltSample_MainProcess(void)
{
    uint32_t ok = 0;
    float rate = 0;
    for(uint16_t i = 0;i < SAMPLE_TIMES;i++)
    {
        Get_PWMC_BusVoltage();
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Sys_AnalogTemp.Regular_RAW, 3);
        if(fabsf((float)(Sys_AnalogTemp.V_Bus - BUS_VOLT_REFERENCE)) < BUS_VOLT_TOLERATE)
            ok++;
    }
    rate = (float)ok / (float)SAMPLE_TIMES;
    if(rate < PASS_RATE)
        result_feedback.bits.Item_BusVoltSample = 0;
    else
        result_feedback.bits.Item_BusVoltSample = 1;
}

void RS485TxRx_MainProcess(void)
{
    timeout_manager.tick_start = SystemTimer_GetTick_ms();
    timeout_manager.tick_end = timeout_manager.tick_start + TIMEOUT_INTERVAL_MS;
    timeout_manager.waiting = true;
    timeout_manager.timeout = false;
    TestItem_t temp = {.raw_value = 0};
    uint16_t len = Pack(ID_Wire, (uint8_t *)uart_buffers.tx_dma_buffer, true, temp, temp);
    MSUART_Start_DMA_Transfer(&msuart3, 1, 0, NULL, (const uint8_t *)uart_buffers.tx_dma_buffer, len, 1, 1, 0, UART_RxCallback, (uint8_t *)uart_buffers.rx_dma_buffer, RAW_FRAME_SIZE_MAX);
}

void InputEncoderSPI_MainProcess(void)
{
    //KTH7816
    uint32_t ok = 0;
    float rate = 0;
    for(uint16_t i = 0;i < KTH7816_SAMPLE_TIMES;i++)
    {
        if(KTH7816_Raw_Pos > 0 && KTH7816_Raw_Pos < 0xFFFF)
            ok++;
    }
    rate = (float)ok / (float)KTH7816_SAMPLE_TIMES;
    if(rate < PASS_RATE)
        result_feedback.bits.Item_InputEncoderSPI = 0;
    else
        result_feedback.bits.Item_InputEncoderSPI = 1;
}

void OutputEncoderI2C_MainProcess(void)
{
    //KTH5701
    uint32_t ok = 0;
    float rate = 0;
    for(uint16_t i = 0;i < KTH5701_SAMPLE_TIMES;i++)
    {
        KTH5701_Async_Read_Measurement(NULL);
        if(KTH5701_Raw_Measurement.TABZ.A > 0 && KTH5701_Raw_Measurement.TABZ.A < 0xFFFF)
            ok++;
		SystemTimer_Delay_ms(1);
    }
    rate = (float)ok / (float)KTH5701_SAMPLE_TIMES;
    if(rate < PASS_RATE)
        result_feedback.bits.Item_OutputEncoderI2C = 0;
    else
        result_feedback.bits.Item_OutputEncoderI2C = 1;
}

void VrefSample_MainProcess(void)
{
    uint32_t ok = 0;
    float rate = 0;
    for(uint16_t i = 0;i < SAMPLE_TIMES;i++)
    {
        Get_MCU_Vref();
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Sys_AnalogTemp.Regular_RAW, 3);
		SystemTimer_Delay_ms(1);
        if(fabsf(Sys_AnalogTemp.Vref - V_INPUT_REFERENCE) < VREF_TOLERATE)
            ok++;
    }   
    rate = (float)ok / (float)SAMPLE_TIMES;
    if(rate < PASS_RATE)
        result_feedback.bits.Item_VrefSample = 0;
    else
        result_feedback.bits.Item_VrefSample = 1;
}

void PhaseCurrentSample_MainProcess(void)
{
    uint32_t ok = 0;
    float rate = 0;
    for(uint16_t i = 0;i < SAMPLE_TIMES;i++)
    {
        Get_PWMC_Current();
        float phase_a_current = Sys_AnalogTemp.Injected_data[0];
        float phase_b_current = Sys_AnalogTemp.Injected_data[1];
        float phase_c_current = Sys_AnalogTemp.Injected_data[2];
        if (fabsf((float)(phase_a_current - PHASE_CURRENT_REFERENCE)) < PHASE_CURRENT_TOLERATE &&
            fabsf((float)(phase_b_current - PHASE_CURRENT_REFERENCE)) < PHASE_CURRENT_TOLERATE &&
            fabsf((float)(phase_c_current - PHASE_CURRENT_REFERENCE)) < PHASE_CURRENT_TOLERATE)
          ok++;
        SystemTimer_Delay_ms(1);
    }
    rate = (float)ok / (float)SAMPLE_TIMES;
    if(rate < PASS_RATE)
        result_feedback.bits.Item_PhaseCurrentSample = 0;
    else
        result_feedback.bits.Item_PhaseCurrentSample = 1;
}

void DRV8316State_MainProcess(void)
{
    uint8_t drv_fault_cnt = 0;
	uint8_t drv_fault_pin_feedback = 0;
	  float rate = 0;
	
		DRV8316_Update_Reg_DMA();
    DRV8316_ICSR_u ICSR = DRV8316_Registers.ICSR;
    DRV8316_SR1_u SR1 = DRV8316_Registers.SR1;
    DRV8316_SR2_u SR2 = DRV8316_Registers.SR2;
		
		 // drv FAULT
		for(uint16_t i = 0;i < SAMPLE_TIMES;i++)
	  {
			if (HAL_GPIO_ReadPin(GPIOB, FAULT_Pin) == RESET)
			{	
        drv_fault_cnt++;
			}
			else if (drv_fault_cnt > 0)
			{
        drv_fault_cnt--;
			}
	  }
		rate = 1.0F - (float)drv_fault_cnt / (float)SAMPLE_TIMES;
    if(rate < PASS_RATE)
		{
        drv_fault_pin_feedback = 1;		
		}
		
	
    if((ICSR.reg_value != 0x08) || SR1.reg_value || SR2.reg_value || DRV8316_SPI_Error_CNT || drv_fault_pin_feedback)
        result_feedback.bits.Item_DRV8316State = 0;
    else
        result_feedback.bits.Item_DRV8316State = 1;
}

void DRV8316Drive_MainProcess(void)
{
    Drive_Reset();
    Switch_Drive_Phase(U_PHASE);
    SystemTimer_Delay_ms(250);
    Switch_Drive_Phase(V_PHASE);
    SystemTimer_Delay_ms(250);
    Switch_Drive_Phase(W_PHASE);
		SystemTimer_Delay_ms(250);
    Drive_Reset();

    if(drive_config.uPassTimes >= DRV8316_U_PASS_TIMES &&
        drive_config.vPassTimes >= DRV8316_V_PASS_TIMES &&
        drive_config.wPassTimes >= DRV8316_W_PASS_TIMES)
    {
        result_feedback.bits.Item_DRV8316Drive = 1;
    }
    else
    {
        result_feedback.bits.Item_DRV8316Drive = 0;
    }
}
