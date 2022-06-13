#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdbool.h>
#include "stm32f4xx_conf.h"
#include "spi.h"
#include "debug_link.h"

#define MS5611_D1_CONVERT_OSR256  0x40
#define MS5611_D1_CONVERT_OSR512  0x42
#define MS5611_D1_CONVERT_OSR1024 0x44
#define MS5611_D1_CONVERT_OSR2048 0x46
#define MS5611_D1_CONVERT_OSR4096 0x48

#define MS5611_D2_CONVERT_OSR256  0x50
#define MS5611_D2_CONVERT_OSR512  0x52
#define MS5611_D2_CONVERT_OSR1024 0x54
#define MS5611_D2_CONVERT_OSR2048 0x56
#define MS5611_D2_CONVERT_OSR4096 0x58

#define MS5611_OSR256_DELAY  0.5
#define MS5611_OSR512_DELAY    1
#define MS5611_OSR1024_DELAY   2
#define MS5611_OSR2048_DELAY   4
#define MS5611_OSR4096_DELAY  10

typedef struct {
	uint32_t c1, c2, c3, c4, c5, c6; //internal calibration datas of ms5611

	int32_t d1, d2;

	float temp_raw;        //raw temperature

	bool init_finished;
	bool velocity_ready;

	float press_raw;       //raw pressure
	float press_lpf;       //low pass filtered pressure
	float press_sea_level; //sea leval presaure value

	float rel_alt;      //relative altitude [m]
	float rel_alt_last; //save for numerical differentiation
	float rel_vel_raw;  //raw data of relative altitude rate [m/s]
	float rel_vel_lpf;  //low pass filtered relative altitude rate [m/s]

	float last_read_time;
	float update_freq;
} ms5611_t;

bool ms5611_available(void);
void ms5611_init(void);
void ms5611_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                          UBaseType_t priority);
void ms5611_wait_until_stable(void);
void ms5611_set_sea_level(void);

float ms5611_get_update_freq(void);
float ms5611_get_pressure(void);
float ms5611_get_relative_altitude(void);
float ms5611_get_relative_altitude_rate(void);

void ms5611_driver_handler(BaseType_t *higher_priority_task_woken);
void ms5611_driver_trigger_handler(void);

void send_barometer_debug_message(debug_msg_t *payload);

#endif

