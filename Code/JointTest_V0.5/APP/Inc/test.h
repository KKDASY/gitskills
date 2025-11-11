#ifndef __TEST_H__
#define __TEST_H__

#include "stm32g431xx.h"
#include "math.h"

#define SAMPLE_TIMES                    (1000)
#define KTH7816_SAMPLE_TIMES            SAMPLE_TIMES
#define KTH5701_SAMPLE_TIMES            SAMPLE_TIMES
#define BUS_VOLT_REFERENCE              (12.0f)
#define V_INPUT_REFERENCE               (3.3f)
#define PASS_RATE                       (0.95f)
#define PHASE_CURRENT_REFERENCE         (0.0f)

#define TIMEOUT_INTERVAL_MS             (2000)
#define BUS_VOLT_TOLERATE               (1.0f)
#define VREF_TOLERATE                   (0.5f)
#define PHASE_CURRENT_TOLERATE          (0.1f)

#define DRV8316_U_PASS_TIMES            (3250)
#define DRV8316_V_PASS_TIMES            (3250)
#define DRV8316_W_PASS_TIMES            (3250)

void Test(void);
void BusVoltSample_MainProcess(void);
void RS485TxRx_MainProcess(void);
void InputEncoderSPI_MainProcess(void);
void OutputEncoderI2C_MainProcess(void);
void VrefSample_MainProcess(void);

#endif
