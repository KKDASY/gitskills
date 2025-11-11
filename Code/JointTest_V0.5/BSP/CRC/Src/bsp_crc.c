#include "bsp_crc.h"

extern CRC_HandleTypeDef hcrc;

void HW_CRC_Init(void)
{
    hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	hcrc.Init.GeneratingPolynomial = 0x8005;    //CRC polynomial for MODBUS
	hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
	hcrc.Init.InitValue = 0xffff;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
}

uint16_t Get_CRC16_MODBUS(const uint8_t *data, uint16_t len) 
{
    uint16_t crc_bytes = (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len);
    if (SWAP_CRC_BYTE) 
    {
        uint16_t crc_temp = (crc_bytes >> 8) | (crc_bytes << 8);
        return crc_temp;
    }
    return crc_bytes;
}  
