#ifndef __TEST_H__
#define __TEST_H__

#include "main.h"
#include "stdbool.h"
#include "bsp_key.h"

#define TIMEOUT_MS 10000
#define KTH5701_SAMPLE_TIMES 1000
#define PASS_RATE (0.95f)

void test(void);
void SpinalTest_MainProcess(TestItem_t test_item);
void DriverTest_MainProcess(TestItem_t test_item);
void WireTest_MainProcess(TestItem_t test_item);
void IMUTest_MainProcess(TestItem_t test_item);
bool OutputEncoderI2C_SubProcess(void);
void RS485TxRx_SubProcess(void);

#endif

