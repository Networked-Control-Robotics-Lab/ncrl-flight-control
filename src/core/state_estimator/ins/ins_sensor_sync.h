#ifndef __INS_SENSOR_SYNC_H__
#define __INS_SENSOR_SYNC_H__

typedef struct {
	float timestamp_s;
	float height;
	float height_rate;
} ins_sync_barometer_item_t;

typedef struct {
	float timestamp_s;
	float longitude;
	float latitude;
	float height_msl;
	float vx_ned;
	float vy_ned;
	float vz_ned;
} ins_sync_gps_item_t;

void ins_sync_buffer_init(void);

void ins_barometer_sync_buffer_push(float height, float height_rate);
bool ins_barometer_sync_buffer_pop(float *height, float *height_rate);

void ins_gps_sync_buffer_push(float longitude, float latitude, float height_msl,
                              float vx_ned, float vy_ned, float vz_ned);
bool ins_gps_sync_buffer_pop(float *longitude, float *latitude, float *height_msl,
                             float *vx_ned, float *vy_ned, float *vz_ned);

#endif
