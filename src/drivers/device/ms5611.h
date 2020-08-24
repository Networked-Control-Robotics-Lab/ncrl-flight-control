#ifndef __MS5611_H__
#define __MS5611_H__

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "debug_link.h"

#define ms5611_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define ms5611_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_15)

typedef struct {
	uint16_t c1, c2, c3, c4, c5, c6; //internal calibration datas of ms5611

	float temp_raw;        //raw temperature

	float press_raw;       //raw pressure
	float press_lpf;       //low pass filtered pressure
	float press_sea_level; //sea leval presaure value

	float rel_alt;      //relative altitude [m]
	float rel_alt_last; //save for numerical differentiation
	float rel_vel_raw;  //raw data of relative altitude rate [m/s]
	float rel_vel_lpf;  //low pass filtered relative altitude rate [m/s]
} ms5611_t;

void ms5611_init(void);
void ms5611_set_sea_level(void);

float ms5611_get_pressure(void);
float ms5611_get_relative_altitude(void);
float ms5611_get_relative_altitude_rate(void);

void ms5611_driver_semaphore_handler(void);
void ms5611_driver_task(void *param);

void send_barometer_debug_message(debug_msg_t *payload);

#endif

