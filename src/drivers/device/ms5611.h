#ifndef __MS5611_H__
#define __MS5611_H__

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "debug_link.h"

#define ms5611_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define ms5611_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_15)

void ms5611_init(void);
void ms5611_read_pressure(void);
float ms5611_get_relative_height(void);
void ms5611_set_sea_level(void);

void send_barometer_debug_message(debug_msg_t *payload);

#endif

