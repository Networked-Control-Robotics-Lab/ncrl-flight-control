#ifndef __IMU_H__
#define __IMU_H__

#include "debug_link.h"

typedef struct {
	int16_t accel_unscaled[3];
	int16_t gyro_unscaled[3];
	int16_t temp_unscaled;

	float accel_raw[3];
	float gyro_raw[3];
	float mag_raw[3];
	float temp_raw;

	float accel_lpf[3];
	float accel_unscaled_lpf[3];
	float gyro_lpf[3];
	float mag_lpf[3];
} imu_t;

void config_imu_accel_scale_calib_setting(float x_scale, float y_scale, float z_scale);
void config_imu_accel_offset_calib_setting(float x_offset, float y_offset, float z_offset);

void get_imu_raw_accel(float *accel);
void get_imu_filtered_accel(float *accel);
float get_imu_accel_update_freq(void);
    
void get_imu_raw_gyro(float *gyro);
void get_imu_filtered_gyro(float *gyro);
float get_imu_gyro_update_freq(void);

bool is_compass_present(void);
void get_imu_compass_raw(float *mag_raw);
float get_imu_compass_raw_strength(void);
float get_imu_compass_update_freq(void);

void send_compass_debug_message(debug_msg_t *payload);

#endif
