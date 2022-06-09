#ifndef __LIDAR_LITE_H__
#define __LIDAR_LITE_H__

#include <stdint.h>
#include <stdbool.h>
#include "debug_link.h"

#define LIDAR_FILTER_SIZE 11 //should be a odd number

#define LIDAR_DEV_ADDRESS 0x62

#define LIDAR_ACQ_COMMAND_REG    0x00
#define LIDAR_STATUS_REG         0x01
#define LIDAR_ACQ_CONFIG_REG     0x04
#define LIDAR_VELOCITY_REG       0x09
#define LIDAR_MEASURE_COUNT_REG  0x11
#define LIDAR_MEASURE_DELAY_REG  0x45
#define LIDAR_DISTANCE_REG       0x8f

typedef struct {
	float height_raw;      //[m]
	float height_filtered; //[m]

	float height_last;
	float vel_raw;  //[m/s]
	int prescaler_cnt;
	float prescaler;
	float dt;

	float last_read_time;
	float update_freq;

	float med_filter_sort[LIDAR_FILTER_SIZE]; //sorted data
	float med_filter_buff[LIDAR_FILTER_SIZE]; //unsorted raw data
	int med_filter_cnt; //number of the data in the filter array
	int med_filter_ptr; //current position to the buffer
} lidar_lite_t;

void lidar_lite_init(void);

void lidar_lite_read_sensor(void);

bool lidar_lite_available();
float lidar_lite_get_update_freq(void);

float lidar_lite_get_distance(void);
float lidar_lite_get_velocity(void);

void send_rangefinder_debug_message(debug_msg_t *payload);

#endif
