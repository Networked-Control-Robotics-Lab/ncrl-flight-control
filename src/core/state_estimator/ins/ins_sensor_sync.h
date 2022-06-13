#ifndef __INS_SENSOR_SYNC_H__
#define __INS_SENSOR_SYNC_H__

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct {
	float timestamp_s;
	float height;
	float height_rate;
} ins_sync_barometer_item_t;

typedef struct {
	float timestamp_s;
	int32_t longitude;
	int32_t latitude;
	float height_msl;
	float vx_ned;
	float vy_ned;
	float vz_ned;
} ins_sync_gps_item_t;

typedef struct {
	float timestamp_s;
	float mag_x;
	float mag_y;
	float mag_z;
} ins_sync_compass_item_t;

void ins_sync_buffer_init(void);
bool ins_sync_buffer_is_ready(void);

typedef struct {
	float timestamp_s;
	float height;
	float height_rate;
} ins_sync_rangefinder_item_t;

bool ins_barometer_sync_buffer_available(void);
void ins_barometer_sync_buffer_push(float height, float height_rate);
void ins_barometer_sync_buffer_push_from_isr(float height, float height_rate,
                BaseType_t *higher_priority_task_woken);
bool ins_barometer_sync_buffer_pop(float *height, float *height_rate);

bool ins_gps_sync_buffer_available(void);
void ins_gps_sync_buffer_push(int32_t longitude, int32_t latitude, float height_msl,
                              float vx_ned, float vy_ned, float vz_ned);
void ins_gps_sync_buffer_push_from_isr(int32_t longitude, int32_t latitude, float height_msl,
                                       float vx_ned, float vy_ned, float vz_ned,
                                       BaseType_t *higher_priority_task_woken);
bool ins_gps_sync_buffer_pop(int32_t *longitude, int32_t *latitude, float *height_msl,
                             float *vx_ned, float *vy_ned, float *vz_ned);

bool ins_compass_sync_buffer_available(void);
void ins_compass_sync_buffer_push(float *mag);
void ins_compass_sync_buffer_push_from_isr(float *mag, BaseType_t *higher_priority_task_woken);
bool ins_compass_sync_buffer_pop(float *mag);

bool ins_rangefinder_sync_buffer_available(void);
void ins_rangefinder_sync_buffer_push(float height, float height_rate);
void ins_rangefinder_sync_buffer_push_from_isr(float height, float height_rate,
                BaseType_t *higher_priority_task_woken);
bool ins_rangefinder_sync_buffer_pop(float *height, float *height_rate);

#endif
