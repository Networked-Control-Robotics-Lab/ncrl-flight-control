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
	int8_t vel_raw;    //[cm/s]

	uint16_t dist_last;
	float vel_num_diff;
	int prescaler_cnt;
	float prescaler;
	float dt;
} lidar_lite_t;

void lidar_lite_init(void);

void lidar_lite_task_handler(void);

float lidar_lite_get_distance(void);
float lidar_lite_get_velocity(void);

void send_rangefinder_debug_message(debug_msg_t *payload);

#endif
