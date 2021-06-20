#ifndef __LIDAR_LITE_H__
#define __LIDAR_LITE_H__

#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"

#define LIDAR_DEV_ADDRESS 0x62

#define LIDAR_ACQ_COMMAND_REG    0x00
#define LIDAR_STATUS_REG         0x01
#define LIDAR_ACQ_CONFIG_REG     0x04
#define LIDAR_VELOCITY_REG       0x09
#define LIDAR_MEASURE_COUNT_REG  0x11
#define LIDAR_MEASURE_DELAY_REG  0x45
#define LIDAR_DISTANCE_REG       0x8f

typedef struct {
	uint16_t dist_raw; //[cm]

	uint16_t dist_last;
	float vel_num_diff; //[cm/s]
	int prescaler_cnt;
	float prescaler;
	float dt;

	float last_read_time;
	float update_freq;
} lidar_lite_t;

void lidar_lite_init(void);

void lidar_lite_task_handler(void);

bool lidar_lite_available();

float lidar_lite_get_distance(void);
float lidar_lite_get_velocity(void);

void send_rangefinder_debug_message(debug_msg_t *payload);

#endif
