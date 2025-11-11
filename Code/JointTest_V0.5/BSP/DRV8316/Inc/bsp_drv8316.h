#ifndef __DRV8316_H
#define __DRV8316_H

#include "stm32g4xx_hal.h"
#include "main.h"
#include "spi.h"
#include "stdbool.h"

#define DRV8316_SPI_DMA_RX_IRQHANDLER       DMA1_Channel4_IRQHandler
#define READ_REG_NUM                        3

#define SPI_SDO_MODE_PUSH_PULL              0x01
#define SPI_SDO_MODE_OPEN_DRAIN             0x00

#define PWM_MODE_6X                         0x00
#define PWM_MODE_3X                         0x02

#define SLEW_RATE_25V_US                    0x00
#define SLEW_RATE_50V_US                    0x01
#define SLEW_RATE_125V_US                   0x02
#define SLEW_RATE_200V_US                   0x03

#define OVP_LEVEL_34V                       0x00
#define OVP_LEVEL_22V                       0x01

#define OCP_LEVEL_16A                       0x00
#define OCP_LEVEL_24A                       0x01

#define OCP_DEGLITCH_0_2US                  0x00
#define OCP_DEGLITCH_0_6US                  0x01
#define OCP_DEGLITCH_1_25US                 0x02
#define OCP_DEGLITCH_1_6US                  0x03

#define CSA_GAIN_0_15V_A                    0x00
#define CSA_GAIN_0_3V_A                     0x01
#define CSA_GAIN_0_6V_A                     0x02
#define CSA_GAIN_1_2V_A                     0x03

#define DLY_TARGET_0US                      0x00
#define DLY_TARGET_0_4US                    0x01
#define DLY_TARGET_0_6US                    0x02
#define DLY_TARGET_0_8US                    0x03
#define DLY_TARGET_1US                      0x04
#define DLY_TARGET_1_2US                    0x05
#define DLY_TARGET_1_4US                    0x06
#define DLY_TARGET_1_6US                    0x07
#define DLY_TARGET_1_8US                    0x08
#define DLY_TARGET_2US                      0x09
#define DLY_TARGET_2_2US                    0x0A
#define DLY_TARGET_2_4US                    0x0B
#define DLY_TARGET_2_6US                    0x0C
#define DLY_TARGET_2_8US                    0x0D
#define DLY_TARGET_3US                      0x0E
#define DLY_TARGET_3_2US                    0x0F

#define CONFIG_PWM_MODE                         PWM_MODE_3X
#define CONFIG_SLEW_RATE                        SLEW_RATE_25V_US
#define CONFIG_OVP_LEVEL                        OVP_LEVEL_22V
#define CONFIG_OTP_ON_NFAULT_PIN                true
#define CONFIG_OCP_LEVEL                        OCP_LEVEL_16A
#define CONFIG_OCP_DEGLITCH                     OCP_DEGLITCH_0_6US
#define CONFIG_SPI_SDO_MODE                     SPI_SDO_MODE_PUSH_PULL
#define CONFIG_CSA_GAIN                         CSA_GAIN_0_15V_A
#define CONFIG_DLY_TARGET                       DLY_TARGET_0US
#define CONFIG_DLYCMP_ENABLE                    false
#define CONFIG_SPIERR_ON_NFAULT_PIN             0x01
#define CONFIG_OVP_ENABLE                       0x01
#define CONFIG_OCP_MODE                         0x00
#define CONFIG_REG_H08_VALUE                    0x0A

#define DRV8316_SPI_HANDLER                     hspi1
#define DRV8316_SPI_INSTANCE                    SPI1
#define DRV8316_SPI_BAUDRATE_PRESCALER          SPI_BAUDRATEPRESCALER_32
#define DRV8316_SPI_DMA_TX_IRQn                 DMA1_Channel3_IRQn
#define DRV8316_SPI_DMA_RX_IRQn                 DMA1_Channel4_IRQn
#define DRV8316_SPI_DMA_RX_IRQHANDLER           DMA1_Channel4_IRQHandler
#define DRV8316_SPI_CS_GPIO_PORT                SPI1_CS_GPIO_Port
#define DRV8316_SPI_CS_GPIO_PIN                 SPI1_CS_Pin
#define DRV8316_NFAULT_GPIO_PORT                FAULT_GPIO_Port
#define DRV8316_NFAULT_GPIO_PIN                 FAULT_Pin
#define DRV8316_OFF_GPIO_PORT                   DRVOFF_GPIO_Port
#define DRV8316_OFF_GPIO_PIN                    DRVOFF_Pin
#define CONFIG_REG_NUM                          (6)

typedef enum
{
    DRV8316_REG_ICSR,
    DRV8316_REG_SR1,
    DRV8316_REG_SR2,
    DRV8316_REG_CR1,
    DRV8316_REG_CR2,
    DRV8316_REG_CR3,
    DRV8316_REG_CR4,
    DRV8316_REG_CR5,
    DRV8316_REG_CR6,
    DRV8316_REG_CR10 = 0x0C
} DRV8316_Reg_e;

typedef union
{
    struct
    {
        uint8_t FAULT       :1;// Bit 0: 0h = No fault, 1h = Fault detected
        uint8_t OT          :1;// Bit 1: 0h = No over-temperature, 1h = Over-temperature detected
        uint8_t OVP         :1;// Bit 2: 0h = No over-voltage, 1h = Over-voltage detected
        uint8_t NPOR        :1;// Bit 3: 0h = No power-on reset, 1h = Power-on reset detected
        uint8_t OCP         :1;// Bit 4: 0h = No over-current, 1h = Over-current detected
        uint8_t SPI_FLT     :1;// Bit 5: 0h = No SPI fault, 1h = SPI fault detected
        uint8_t BK_FLT      :1;// Bit 6: 0h = No back-EMF fault, 1h = Back-EMF fault detected
        uint8_t RSVD        :1;// Bit 7: Reserved
    }bit_value;
    uint8_t reg_value;
}DRV8316_ICSR_u;

typedef union
{
    struct
    {
        uint8_t OCP_LA : 1;  // Bit 0: 0h = No overcurrent on low-side OUTA, 1h = Overcurrent detected
        uint8_t OCP_HA : 1;  // Bit 1: 0h = No overcurrent on high-side OUTA, 1h = Overcurrent detected
        uint8_t OCP_LB : 1;  // Bit 2: 0h = No overcurrent on low-side OUTB, 1h = Overcurrent detected
        uint8_t OCP_HB : 1;  // Bit 3: 0h = No overcurrent on high-side OUTB, 1h = Overcurrent detected
        uint8_t OCP_LC : 1;  // Bit 4: 0h = No overcurrent on low-side OUTC, 1h = Overcurrent detected
        uint8_t OCP_HC : 1;  // Bit 5: 0h = No overcurrent on high-side OUTC, 1h = Overcurrent detected
        uint8_t OTS    : 1;  // Bit 6: 0h = No overtemperature shutdown, 1h = Shutdown detected
        uint8_t OTW    : 1;  // Bit 7: 0h = No overtemperature warning, 1h = Warning detected
    } bit_value;
    uint8_t reg_value;
}DRV8316_SR1_u;

typedef union
{
    struct
    {
        uint8_t SPI_ADDR_FLT : 1;  // Bit 0: 0h = No SPI address fault, 1h = Address fault detected
        uint8_t SPI_SCLK_FLT : 1;  // Bit 1: 0h = No SPI clock framing error, 1h = Framing error detected
        uint8_t SPI_PARITY   : 1;  // Bit 2: 0h = No SPI parity error, 1h = Parity error detected
        uint8_t VCP_UV       : 1;  // Bit 3: 0h = No charge pump undervoltage, 1h = Undervoltage detected
        uint8_t BUCK_UV      : 1;  // Bit 4: 0h = No buck undervoltage, 1h = Undervoltage detected
        uint8_t BUCK_OCP     : 1;  // Bit 5: 0h = No buck overcurrent, 1h = Overcurrent detected
        uint8_t OTP_ERR      : 1;  // Bit 6: 0h = No OTP error, 1h = OTP error detected
        uint8_t RSVD         : 1;  // Bit 7: Reserved
    } bit_value;
    uint8_t reg_value;
}DRV8316_SR2_u;

typedef union
{
    struct
    {
        uint8_t REG_LOCK : 3;  // Bits 0-2: 0h-2h = No effect, 3h = Unlock, 6h = Lock settings
        uint8_t RSVD     : 5;  // Bits 3-7: Reserved
    } bit_value;
    uint8_t reg_value;
}DRV8316_CR1_u;

typedef union
{
    struct
    {
        uint8_t CLR_FLT  : 1;  // Bit 0: 0h = No clear fault, 1h = Clear latched faults
        uint8_t PWM_MODE : 2;  // Bits 1-2: 0h = 6x mode, 1h = 6x mode with limit, 2h = 3x mode, 3h = 3x mode with limit
        uint8_t SLEW     : 2;  // Bits 3-4: 0h = 25 V/μs, 1h = 50 V/μs, 2h = 125 V/μs, 3h = 200 V/μs
        uint8_t SDO_MODE : 1;  // Bit 5: 0h = Open Drain, 1h = Push Pull
        uint8_t RSVD     : 2;  // Bits 6-7: Reserved
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR2_u;

typedef union
{
    struct
    {
        uint8_t OTW_REP      : 1;  // Bit 0: 0h = Over temperature reporting on nFAULT is disabled, 1h = Over temperature reporting on nFAULT is enabled
        uint8_t SPI_FLT_REP  : 1;  // Bit 1: 0h = SPI fault reporting on nFAULT pin is enabled, 1h = SPI fault reporting on nFAULT pin is disabled
        uint8_t OVP_EN       : 1;  // Bit 2: 0h = Overvoltage protection is disabled, 1h = Overvoltage protection is enabled
        uint8_t OVP_SEL      : 1;  // Bit 3: 0h = VM overvoltage level is 34-V, 1h = VM overvoltage level is 22-V
        uint8_t PWM_100_DUTY_SEL : 1;  // Bit 4: 0h = 20KHz, 1h = 40KHz
        uint8_t RSVD1        : 1;  // Bit 5: Reserved
        uint8_t RSVD2        : 1;  // Bit 6: Reserved
        uint8_t RSVD3        : 1;  // Bit 7: Reserved
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR3_u;

typedef union
{
    struct
    {
        uint8_t OCP_MODE : 2;  // Bits 0-1: 0h = Overcurrent causes a latched fault, 1h = Overcurrent causes an automatic retrying fault, 2h = Overcurrent is report only but no action is taken, 3h = Overcurrent is not reported and no action is taken
        uint8_t OCP_LVL  : 1;  // Bit 2: 0h = OCP level is 16 A, 1h = OCP level is 24 A
        uint8_t OCP_RETRY: 1;  // Bit 3: 0h = OCP retry time is 5 ms, 1h = OCP retry time is 500 ms
        uint8_t OCP_DEG  : 2;  // Bits 4-5: 0h = OCP deglitch time is 0.2 μs, 1h = OCP deglitch time is 0.6 μs, 2h = OCP deglitch time is 1.25 μs, 3h = OCP deglitch time is 1.6 μs
        uint8_t OCP_CBC  : 1;  // Bit 6: 0h = OCP clearing in PWM input cycle change is disabled, 1h = OCP clearing in PWM input cycle change is enabled
        uint8_t DRV_OFF  : 1;  // Bit 7: 0h = No Action, 1h = Hi-Z FETs
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR4_u;

typedef union
{
    struct
    {
        uint8_t CSA_GAIN  : 2;  // Bits 0-1: 0h = CSA gain is 0.15 V/A, 1h = CSA gain is 0.3 V/A, 2h = CSA gain is 0.6 V/A, 3h = CSA gain is 1.2 V/A
        uint8_t EN_ASR    : 1;  // Bit 2: 0h = ASR mode is disabled, 1h = ASR mode is enabled
        uint8_t EN_AAR    : 1;  // Bit 3: 0h = AAR mode is disabled, 1h = AAR mode is enabled
        uint8_t RSVD1     : 1;  // Bit 4: Reserved
        uint8_t RSVD2     : 1;  // Bit 5: Reserved
        uint8_t ILIM_RECIR: 1;  // Bit 6: 0h = Current recirculation through FETs (Brake Mode), 1h = Current recirculation through diodes (Coast Mode)
        uint8_t RSVD3     : 1;  // Bit 7: Reserved
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR5_u;

typedef union
{
    struct
    {
        uint8_t BUCK_DIS   : 1;  // Bit 0: 0h = Buck regulator is enabled, 1h = Buck regulator is disabled
        uint8_t BUCK_SEL   : 2;  // Bits 1-2: 0h = Buck voltage is 3.3 V, 1h = Buck voltage is 5.0 V, 2h = Buck voltage is 4.0 V, 3h = Buck voltage is 5.7 V
        uint8_t BUCK_CL    : 1;  // Bit 3: 0h = Buck regulator current limit is set to 600 mA, 1h = Buck regulator current limit is set to 150 mA
        uint8_t BUCK_PS_DIS: 1;  // Bit 4: 0h = Buck power sequencing is enabled, 1h = Buck power sequencing is disabled
        uint8_t RSVD1      : 1;  // Bit 5: Reserved
        uint8_t RSVD2      : 2;  // Bits 6-7: Reserved
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR6_u;

typedef union
{
    struct
    {
        uint8_t DLY_TARGET: 4;  // Bits 0-3: 0h = 0 us, 1h = 0.4 us, 2h = 0.6 us, 3h = 0.8 us, 4h = 1 us, 5h = 1.2 us, 6h = 1.4 us, 7h = 1.6 us, 8h = 1.8 us, 9h = 2 us, Ah = 2.2 us, Bh = 2.4 us, Ch = 2.6 us, Dh = 2.8 us, Eh = 3 us, Fh = 3.2 us
        uint8_t DLYCMP_EN : 1;  // Bit 4: 0h = Disable, 1h = Enable
        uint8_t RSVD      : 3;  // Bits 5-7: Reserved
    } bit_value;
    uint8_t reg_value;
} DRV8316_CR10_u;

typedef struct
{
    DRV8316_CR2_u CR2;
    DRV8316_CR3_u CR3;
    DRV8316_CR4_u CR4;
    DRV8316_CR5_u CR5;
    DRV8316_CR6_u CR6;
    DRV8316_CR10_u CR10;
}DRV8316_RegConfig_t;

typedef struct
{
    DRV8316_ICSR_u ICSR;
    DRV8316_SR1_u SR1;
    DRV8316_SR2_u SR2;
    DRV8316_CR1_u CR1;
    DRV8316_CR2_u CR2;
    DRV8316_CR3_u CR3;
    DRV8316_CR4_u CR4;
    DRV8316_CR5_u CR5;
    DRV8316_CR6_u CR6;
    DRV8316_CR10_u CR10;
}DRV8316_Reg_t;

void DRV8316_Init(void);
bool DRV8316_Config_Registers(void);
void DRV8316_Update_Reg_DMA(void);

#endif
