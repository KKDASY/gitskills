#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "stm32g4xx_hal.h"
#include "main.h"
#include "tim.h"

#define MOTOR_RATED_VOLTAGE_MAX (20)
#define MOTOR_RATED_VOLTAGE_MIN (12)

#define ADC_REF_V                   (float)(3.3f) 
#define VBUS_UP_RES                 (float)(68.0f)
#define VBUS_DOWN_RES               (float)(5.10f) 
#define VBUS_CONVERSION_FACTOR      (float)(ADC_REF_V*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/4095.0f)
#define SAMPLE_CURR_CON_FACTOR      (double)(0.0053724)   // 0.15V /A

#define READ_IPHASE_A_ADC() 		((uint16_t)ADC1->JDR4)
#define READ_IPHASE_C_ADC()  		((uint16_t)ADC1->JDR3)

typedef struct sAnalogValue
{
	uint16_t Regular_RAW[4];//MCU_VREF+MCU_Temp+VBUS+NTC MOS
	uint16_t Injected_RAW[4];
	
	float Vref;
	
	float MCU_Temp;
	float Power_Temp;
	float Motor_Temp;
	
	float V_Bus;
	float Injected_data[3];//A/B/C三相浮点电流值
	float Phase_CurrentRMS;	// [A]	相电流有效值	
	
} tAnalogValue;

void ADC_Init(void);
void Get_MCU_Vref(void);
void Get_MCU_Temp(void);
void Get_PWMC_BusVoltage(void);
void PhaseCurrentSample_MainProcess(void);
void Get_PWMC_Current(void);
int PWMC_CurrentReadingPolarization(void);
void DRV8316State_MainProcess(void);
void DRV8316Drive_MainProcess(void);
void Judge_Drive_PhaseCurrent_NG(void);

#endif
