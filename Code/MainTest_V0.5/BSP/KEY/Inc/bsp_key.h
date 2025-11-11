#ifndef KEY_H
#define KEY_H

#include "stdint.h"

typedef enum 
{
    TestedObj_None,
    TestedObj_Spinal,
    TestedObj_Driver,
    TestedObj_MAWire,
    TestedObj_NMAWire,
    TestedObj_IMU
} TestedObj_e;

#define ITEM_BUS_VOLT_BIT 0
#define ITEM_RS485_BIT 1
#define ITEM_INPUT_ENCODER_BIT 2
#define ITEM_OUTPUT_ENCODER_BIT 3
#define ITEM_VREF_BIT 4
#define ITEM_PHASE_CURRENT_BIT 5
#define ITEM_DRV8316_STATE_BIT 6
#define ITEM_DRV8316_DRIVE_BIT 7
#define ITEM_FDCAN_BIT 8
#define ITEM_USB_BIT 9
#define ITEM_PHY_VOLT_BIT 10
#define ITEM_IMUSPI_BIT 11

typedef union
{
    uint16_t raw_value;
    struct
    {
        uint8_t Item_BusVoltSample : 1;
        uint8_t Item_RS485TxRx : 1;
        uint8_t Item_InputEncoderSPI : 1;
        uint8_t Item_OutputEncoderI2C : 1;
        uint8_t Item_VrefSample : 1;
        uint8_t Item_PhaseCurrentSample : 1;
        uint8_t Item_DRV8316State : 1;
        uint8_t Item_DRV8316Drive : 1;
        uint8_t Item_FDCAN : 1;
        uint8_t Item_USB : 1;
        uint8_t Item_PHYVoltSample : 1;
        uint8_t Item_IMUSPI : 1;
        uint8_t Item_Reserved : 4;
    }bits;
}TestItem_t;


void Read_Toggle_Switch(void);
void TestItem_Config(TestedObj_e obj, TestItem_t* test_item);
#endif





