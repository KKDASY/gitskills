/**
*****************************************************************************************
 * @file     kth5701.h
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
---- Define to Prevent Recursive Inclusion ------------------------------------------------------*/

#ifndef KTH5701_H
#define KTH5701_H

#ifdef __cplusplus
    extern "C" {
#endif

/**************************************************************************************************
---- Includes -----------------------------------------------------------------------------------*/

#include "kth5701_cfg.h"
#include "main.h"
#include <stdbool.h>

/**************************************************************************************************
---- Constant Defines ---------------------------------------------------------------------------*/


/**************************************************************************************************
---- Macro Defines ------------------------------------------------------------------------------*/

// ºêº¯Êý  #define GEN_ARRAY(x)  int a[x] = {}

/**************************************************************************************************
---- Type Defines -------------------------------------------------------------------------------*/


/**************************************************************************************************
---- Enumerations -------------------------------------------------------------------------------*/

typedef enum
{
    KTH5701_REG_H06             = 0x06,
    KTH5701_REG_H0D             = 0x0D,
    KTH5701_REG_H14             = 0x14,
    KTH5701_REG_H15             = 0x15,
    KTH5701_REG_H16             = 0x16,
    KTH5701_REG_H17             = 0x17,
    KTH5701_REG_H18             = 0x18,
    KTH5701_REG_H19             = 0x19,
    KTH5701_REG_H1A             = 0x1A,
    KTH5701_REG_H1B             = 0x1B,
    KTH5701_REG_H1C             = 0x1C,
    KTH5701_REG_H1D             = 0x1D,
    KTH5701_REG_H1E             = 0x1E,
    KTH5701_REG_H1F             = 0x1F,
    
} KTH5701_Reg_E;

typedef enum
{
    KTH5701_MODE_IDLE           = 0x00,
    KTH5701_MODE_CONTINUOUS     = 0x01,
    KTH5701_MODE_WAKE_UP_SLEEP  = 0x02,
    KTH5701_MODE_SINGLE         = 0x03,
		KTH5701_MODE_UNKNOW         = 0x0F
    
} KTH5701_Mode_E;

typedef enum
{
    KTH5701_MEASURE_TXYZ        = 0x00,
    KTH5701_MEASURE_TABZ        = 0x01,

} KTH5701_Measure_Type_E;

/**************************************************************************************************
---- Structures ---------------------------------------------------------------------------------*/

typedef union
{
    struct
    {
        uint16_t DRDY        : 1; // Set to 1 after each measurement in Continuous or Single mode, cleared after data read.
        uint16_t SOFT_RST    : 1; // Set to 1 after reset command, cleared after first status read post-reset.
        uint16_t MAGN_DET    : 1; // Set to 1 if magnetic field exceeds threshold set in register 0x1F.
        uint16_t BUTT_DET    : 1; // Set to 1 if button function is detected, based on register 0x1F settings.
        uint16_t FAILING     : 1; // Set to 1 if an invalid command is sent or during incorrect operations.
        uint16_t SINGLE      : 1; // Set to 1 when entering Single Conversion mode.
        uint16_t WAKE_UP     : 1; // Set to 1 when entering Wake-up & Sleep mode.
        uint16_t CONTINUOUS  : 1; // Set to 1 when entering Continuous Sensing mode.
        uint16_t RSVD        : 8;

    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H06_U;

typedef struct
{
    uint16_t CHIP_ID;
    
} KTH5701_Reg_H0D_T;

typedef struct
{
    uint16_t OFFSET_X;
    
} KTH5701_Reg_H14_T;

typedef struct
{
    uint16_t OFFSET_Y;
    
} KTH5701_Reg_H15_T;

typedef struct
{
    uint16_t OFFSET_Z;
    
} KTH5701_Reg_H16_T;

typedef union
{
    struct
    {
        uint16_t SENSXY_LT : 8; // RW Sensitivity calibration for X and Y axes when temperature < tref.
        uint16_t SENSXY_HT : 8; // RW Sensitivity calibration for X and Y axes when temperature > tref.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H17_U;

typedef union
{
    struct
    {
        uint16_t SENSZ_LT : 8; // RW Sensitivity calibration for Z axis when temperature < tref.
        uint16_t SENSZ_HT : 8; // RW Sensitivity calibration for Z axis when temperature > tref.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H18_U;

typedef struct
{
    uint16_t WXY_TH;
    
} KTH5701_Reg_H19_T;

typedef struct
{
    uint16_t WZ_TH;
    
} KTH5701_Reg_H1A_T;

typedef union
{
    struct
    {
        uint16_t GAIN_VALUE : 14; // RW Gain value for amplitude correction, calculated as k*8192.
        uint16_t GAIN_SEL   : 2;  // RW Gain selection for axis correction: 0x00=none, 0x01=X, 0x10=Y, 0x11=Z.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H1B_U;

typedef union
{
    struct
    {
        uint16_t DIG_CTRL   : 3;  // RW Digital filter control parameters.
        uint16_t GAIN       : 4;  // RW Gain control register, recommended default is 0x6.
        uint16_t EXT_TRIG   : 1;  // RW External trigger for single measurement when TRIG_PUSH_SEL=0.
        uint16_t TRIG_PUSH_SEL : 1; // RW Trigger push select for button detection or single measurement.
        uint16_t MAGN_OSR   : 2;  // RW Oversampling rate for magnetic field measurement.
        uint16_t TEMP_OSR   : 2;  // RW Oversampling rate for temperature measurement.
        uint16_t WAKE_SEL   : 1;  // RW Wake-up mode selection.
        uint16_t APLANE_SEL : 2;  // RW Angle output plane selection.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H1C_U;

typedef union
{
    struct
    {
        uint16_t MEAS_TIME : 6;  // RW Controls the standby duration between measurements in continuous and wake-up modes.
        uint16_t MEAS_SEL  : 4;  // RW Measurement selection signal for default measurement gating.
        uint16_t TCMP_EN   : 1;  // RW Temperature compensation enable for magnetic field output.
        uint16_t WAKE_DIFF : 1;  // RW Wake-up mode difference detection enable.
        uint16_t ABS_PUSH_EN : 1; // RW Absolute magnetic field detection for button function.
        uint16_t Z_POL     : 1;  // RW Z-axis polarity control.
        uint16_t Y_POL     : 1;  // RW Y-axis polarity control.
        uint16_t X_POL     : 1;  // RW X-axis polarity control.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H1D_U;

typedef union
{
    struct
    {
        uint16_t ZERO           : 15; // RW Zero point value for angle output, inverted and incremented by 1.
        uint16_t ANG_MAGN_SEL   : 1;  // RW Selects data read frame output: 1=ZBAT, 0=ZYXT.
    } bit_value;

    uint16_t reg_value;

} KTH5701_Reg_H1E_U;

typedef struct
{
    uint16_t PUSH_CONFIG;
    
} KTH5701_Reg_H1F_T;

typedef struct
{
    KTH5701_Reg_H06_U reg_H06;
    KTH5701_Reg_H0D_T reg_H0D;
    KTH5701_Reg_H14_T reg_H14;
    KTH5701_Reg_H15_T reg_H15;
    KTH5701_Reg_H16_T reg_H16;
    KTH5701_Reg_H17_U reg_H17;
    KTH5701_Reg_H18_U reg_H18;
    KTH5701_Reg_H19_T reg_H19;
    KTH5701_Reg_H1A_T reg_H1A;
    KTH5701_Reg_H1B_U reg_H1B;
    KTH5701_Reg_H1C_U reg_H1C;
    KTH5701_Reg_H1D_U reg_H1D;
    KTH5701_Reg_H1E_U reg_H1E;
    KTH5701_Reg_H1F_T reg_H1F;

} KTH5701_Reg_T;

typedef struct
{
    struct
    {
        uint16_t T;
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } TXYZ;

    struct
    {
        uint16_t T;
        uint16_t A;
        uint16_t B;
        uint16_t Z;
    } TABZ;

} KTH5701_Raw_T;

/**************************************************************************************************
---- Extern Variables ---------------------------------------------------------------------------*/

extern volatile KTH5701_Mode_E KTH5701_Mode;
extern KTH5701_Reg_T KTH5701_Registers;
extern KTH5701_Raw_T KTH5701_Raw_Measurement;
extern uint32_t KTH5701_Error_Counter;

/**************************************************************************************************
---- Function Prototypes ------------------------------------------------------------------------*/

void KTH5701_Init(void);
bool KTH5701_Config_Registers(void);

bool KTH5701_Block_Switch_Mode(KTH5701_Mode_E mode);
bool KTH5701_Block_Switch_Measurement_Mode(KTH5701_Measure_Type_E mtype);
bool KTH5701_Block_Reset(void);

bool KTH5701_Block_Read_Reg(KTH5701_Reg_E reg_addr, uint16_t *read_value);
bool KTH5701_Block_Write_Reg(KTH5701_Reg_E reg_addr, uint16_t write_value);

bool KTH5701_Async_Read_Reg(KTH5701_Reg_E reg_addr, void (*read_cplt_cb)(uint16_t read_value));
bool KTH5701_Async_Write_Reg(KTH5701_Reg_E reg_addr, uint16_t write_value, void (*write_cplt_cb)(void));
bool KTH5701_Async_Read_Measurement(void (*read_cplt_cb)(void));  // Call this function cyclically to update KTH5701_Raw_Measurement

void KTH5701_I2C_Mem_Rx_Cplt_Callback_Process(void);        // Put in HAL_I2C_MemRxCpltCallback
void KTH5701_I2C_Master_Tx_Cplt_Callback_Process(void);     // Put in HAL_I2C_MasterTxCpltCallback
void KTH5701_I2C_Master_Rx_Cplt_Callback_Process(void);     // Put in HAL_I2C_MasterRxCpltCallback

/*-----------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* KTH5701_H */

/**** END OF FILE ******************* (C) WUJITECH 2019-2025 ******************** END OF FILE ****/
