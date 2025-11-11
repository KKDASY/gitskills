#include "bsp_adc.h"
#include "kth7816.h"
#include "adc.h"
#include "Motor.h"

extern Drive_t drive_config;

tAnalogValue Sys_AnalogTemp;

int TS_CAL1_30 = 0;   // 温度为30度时，ADC采样值
int TS_CAL2_110 = 0;  // 温度为130度时，ADC采样值
int VREF_CAL_300 = 0;  // 参考电压3V时候的校准值

int16_t phase_a_adc_offset = 0;
int16_t phase_b_adc_offset = 0;
int16_t phase_c_adc_offset = 0;

void ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Sys_AnalogTemp.Regular_RAW, 3);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    TS_CAL1_30 = *TEMPSENSOR_CAL1_ADDR;
    TS_CAL2_110 = *TEMPSENSOR_CAL2_ADDR;
    VREF_CAL_300 = *VREFINT_CAL_ADDR;

    Sys_AnalogTemp.Vref = 3.3;
    Sys_AnalogTemp.MCU_Temp = 25.0f;
    Sys_AnalogTemp.Power_Temp = 25.0f;
    Sys_AnalogTemp.Motor_Temp = 25.0f;
    Sys_AnalogTemp.V_Bus = MOTOR_RATED_VOLTAGE_MIN;

    if(PWMC_CurrentReadingPolarization() != 0)
    {

    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC))
        {
            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
            Get_PWMC_Current();
            Judge_Drive_PhaseCurrent_NG();
			KTH7816_Start_Read_Pos_DMA();
        }
    }
}

void Get_MCU_Vref(void)
{
    uint32_t T_ADC = Sys_AnalogTemp.Regular_RAW[0];
    if(T_ADC != 0)
        Sys_AnalogTemp.Vref = (float)VREFINT_CAL_VREF / 1000 * VREF_CAL_300 / T_ADC;
    else
        Sys_AnalogTemp.Vref = 0;
}

void Get_MCU_Temp(void)
{
    static float last_temp = 25.0f;
    int T_ADC = Sys_AnalogTemp.Regular_RAW[1] * 3.3 / ((float)TEMPSENSOR_CAL_VREFANALOG / 1000);

    float now_temp = (float)((T_ADC - TS_CAL1_30) * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)) / ((float)(TS_CAL2_110 - TS_CAL1_30)) + TEMPSENSOR_CAL1_TEMP;
    Sys_AnalogTemp.MCU_Temp = (last_temp + now_temp) / 2;
    last_temp = now_temp;
}

void Get_PWMC_BusVoltage(void)
{
	Sys_AnalogTemp.V_Bus = Sys_AnalogTemp.Regular_RAW[2] * VBUS_CONVERSION_FACTOR;	
}


void Get_PWMC_Current(void)
{
    Sys_AnalogTemp.Injected_RAW[0] = ADC1->JDR1;
    Sys_AnalogTemp.Injected_RAW[1] = ADC1->JDR2;
    Sys_AnalogTemp.Injected_RAW[2] = ADC1->JDR3;
    Sys_AnalogTemp.Injected_RAW[3] = ADC1->JDR4;

    Sys_AnalogTemp.Injected_data[0] = -(float)(Sys_AnalogTemp.Injected_RAW[0] - phase_a_adc_offset) * SAMPLE_CURR_CON_FACTOR;
    Sys_AnalogTemp.Injected_data[2] = -(float)(Sys_AnalogTemp.Injected_RAW[2] - phase_c_adc_offset) * SAMPLE_CURR_CON_FACTOR;
	

	
	Sys_AnalogTemp.Injected_data[1] = -(Sys_AnalogTemp.Injected_data[0]+Sys_AnalogTemp.Injected_data[2]);
}

void Judge_Drive_PhaseCurrent_NG(void)
{
    if(drive_config.duty_u || drive_config.duty_v || drive_config.duty_w)
    {

        if(Sys_AnalogTemp.Injected_data[0] < drive_config.uPhaseCurrentThresMax &&
            Sys_AnalogTemp.Injected_data[0] > drive_config.uPhaseCurrentThresMin &&
            Sys_AnalogTemp.Injected_data[2] < drive_config.wPhaseCurrentThresMax &&
            Sys_AnalogTemp.Injected_data[2] > drive_config.wPhaseCurrentThresMin 
					)
        {
            if(drive_config.duty_u)
                drive_config.uPassTimes++;
            else if(drive_config.duty_v)
                drive_config.vPassTimes++;
            else if(drive_config.duty_w)
                drive_config.wPassTimes++;
        }
    }
}

uint8_t adc_start = 0;
int PWMC_CurrentReadingPolarization(void)
{
	adc_start = 1;
	int i = 0;
	int adc_sum_a = 0;
	int adc_sum_b = 0;
	int adc_sum_c = 0;
    uint16_t last_vadc1 = 0;

	__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
	while(RESET == __HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_UPDATE)){};
	__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);

	while(i < 1024) //累加求偏移值
	{
		while(SET == __HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_UPDATE))
		{
			__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);

			i ++;

            uint16_t vadc1 = READ_IPHASE_A_ADC();
            uint16_t vadc2 = READ_IPHASE_C_ADC();
            if(vadc1 < 5)
            {
                vadc1 = last_vadc1;
            }
            else
            {
                last_vadc1 = vadc1;
            }
			adc_sum_a += vadc1;
			adc_sum_c += vadc2;
		}
	}

	phase_a_adc_offset = adc_sum_a/i;
	phase_c_adc_offset = adc_sum_c/i;

	// offset check
	i = 0;
	const int Vout = 2048;
	const int check_threshold = 200;
	if(phase_a_adc_offset > (Vout + check_threshold) || phase_a_adc_offset < (Vout - check_threshold))
	{
		i = -1;
	}
	if(phase_c_adc_offset > (Vout + check_threshold) || phase_c_adc_offset < (Vout - check_threshold))
	{
		i = -1;
	}

	return i;
}
