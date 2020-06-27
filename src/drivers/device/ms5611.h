#ifndef __MS5611_H__
#define __MS5611_H__

#include "stm32f4xx_conf.h"
#include "spi.h"

#define ms5611_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define ms5611_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_15)

void ms5611_init(void);
void ms5611_read_pressure(float *temp, float *pressure);

#endif
