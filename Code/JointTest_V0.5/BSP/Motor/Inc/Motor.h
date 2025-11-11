#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32g431xx.h"
#include "tim.h"
#include "adc.h"
#include "stdbool.h"

// #define GREEN_BOARD

#ifdef GREEN_BOARD
#define DUTY_U_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_U_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_U_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_U_PHASE_W_THRES_MAX              ( 0.5f)
#define DUTY_V_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_V_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_V_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_V_PHASE_W_THRES_MAX              ( 0.5f)
#define DUTY_W_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_W_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_W_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_W_PHASE_W_THRES_MAX              ( 0.5f)

#else
#define DUTY_U_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_U_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_U_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_U_PHASE_W_THRES_MAX              ( 0.5f)
#define DUTY_V_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_V_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_V_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_V_PHASE_W_THRES_MAX              ( 0.5f)
#define DUTY_W_PHASE_U_THRES_MIN              (-0.5f)
#define DUTY_W_PHASE_U_THRES_MAX              ( 0.5f)
#define DUTY_W_PHASE_W_THRES_MIN              (-0.5f)
#define DUTY_W_PHASE_W_THRES_MAX              ( 0.5f)
#endif

#define DUTY_STEP                              (0.1f)

typedef enum
{
    U_PHASE,
    V_PHASE,
    W_PHASE,
}Drive_Phase_t;

typedef struct
{
    bool duty_u;
    bool duty_v;
    bool duty_w;

    float uPhaseCurrentThresMin;
    float uPhaseCurrentThresMax;
    float vPhaseCurrentThresMin;
    float vPhaseCurrentThresMax;
    float wPhaseCurrentThresMin;
    float wPhaseCurrentThresMax;

    uint16_t uPassTimes;
    uint16_t vPassTimes;
    uint16_t wPassTimes;
}Drive_t;

void Motor_Enable(void);
void Motor_Disable(void);
void Set_PWM_Duty(float Duty_U, float Duty_V, float Duty_W);
void Drive_Reset(void);
void Switch_Phase_U(void);
void Switch_Phase_V(void);
void Switch_Phase_W(void);
void Switch_Drive_Phase(Drive_Phase_t phase);

#endif
