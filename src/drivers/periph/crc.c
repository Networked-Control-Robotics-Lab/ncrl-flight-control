#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_crc.h"

void _crc_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
}

uint32_t calculate_crc_of_words(uint32_t *data_arr, int size)
{
	CRC_ResetDR(); //clear data register before calculating crc
	return 	CRC_CalcBlockCRC(data_arr, size);
}
