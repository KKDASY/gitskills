#include "Motor.h"

Drive_t drive_config = {0};

void Motor_Enable(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim1);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    Set_PWM_Duty(0, 0, 0);
}

void Motor_Disable(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

void Set_PWM_Duty(float Duty_U, float Duty_V, float Duty_W)
{
__disable_irq();
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)(Duty_W * htim1.Instance->ARR));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint32_t)(Duty_U * htim1.Instance->ARR));
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)(Duty_V * htim1.Instance->ARR));
__enable_irq();
}

void Drive_Reset(void)
{
    drive_config.duty_u = false;
    drive_config.duty_v = false;
    drive_config.duty_w = false;

    drive_config.uPhaseCurrentThresMin = 0;
    drive_config.uPhaseCurrentThresMax = 0;
    drive_config.vPhaseCurrentThresMin = 0;
    drive_config.vPhaseCurrentThresMax = 0;
    drive_config.wPhaseCurrentThresMin = 0;
    drive_config.wPhaseCurrentThresMax = 0;
}

void Switch_Drive_Phase(Drive_Phase_t phase)
{
    switch(phase)
    {
        case U_PHASE:
            Set_PWM_Duty(DUTY_STEP, 0, 0);
            Switch_Phase_U();
            break;
        case V_PHASE:
            Set_PWM_Duty(0, DUTY_STEP, 0);
            Switch_Phase_V();
            break;
        case W_PHASE:
            Set_PWM_Duty(0, 0, DUTY_STEP);
            Switch_Phase_W();
            break;
        default:
            break;
    }
}

void Switch_Phase_U(void)
{
    drive_config.duty_u = true;
    drive_config.duty_v = false;
    drive_config.duty_w = false;

    drive_config.uPhaseCurrentThresMin = DUTY_U_PHASE_U_THRES_MIN;
    drive_config.uPhaseCurrentThresMax = DUTY_U_PHASE_U_THRES_MAX;
    drive_config.wPhaseCurrentThresMin = DUTY_U_PHASE_W_THRES_MIN;
    drive_config.wPhaseCurrentThresMax = DUTY_U_PHASE_W_THRES_MAX;

    drive_config.uPassTimes = 0;
}

void Switch_Phase_V(void)
{
    drive_config.duty_u = false;
    drive_config.duty_v = true;
    drive_config.duty_w = false;

    drive_config.uPhaseCurrentThresMin = DUTY_V_PHASE_U_THRES_MIN;
    drive_config.uPhaseCurrentThresMax = DUTY_V_PHASE_U_THRES_MAX;
    drive_config.wPhaseCurrentThresMin = DUTY_V_PHASE_W_THRES_MIN;
    drive_config.wPhaseCurrentThresMax = DUTY_V_PHASE_W_THRES_MAX;

    drive_config.vPassTimes = 0;
}

void Switch_Phase_W(void)
{
    drive_config.duty_u = false;
    drive_config.duty_v = false;
    drive_config.duty_w = true;

    drive_config.uPhaseCurrentThresMin = DUTY_W_PHASE_U_THRES_MIN;
    drive_config.uPhaseCurrentThresMax = DUTY_W_PHASE_U_THRES_MAX;
    drive_config.wPhaseCurrentThresMin = DUTY_W_PHASE_W_THRES_MIN;
    drive_config.wPhaseCurrentThresMax = DUTY_W_PHASE_W_THRES_MAX;

    drive_config.wPassTimes = 0;
}
