#include "bsp_kth5701.h"
#include "kth5701.h"

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Mem_Rx_Cplt_Callback_Process();
	
	KTH5701_Error_Counter = 0;
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Master_Tx_Cplt_Callback_Process();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	KTH5701_I2C_Master_Rx_Cplt_Callback_Process();
	
	KTH5701_Error_Counter = 0;
}
