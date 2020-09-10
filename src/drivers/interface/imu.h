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

bool imu_calibration_not_finished(void);

void set_accel_scale_factor(float x_scale, float y_scale, float z_scale);
void set_accel_bias(float x_offset, float y_offset, float z_offset);

void reset_accel_scale_factor(void);
void reset_accel_bias(void);

void get_accel_raw(float *accel);
void get_accel_lpf(float *accel);
float get_accel_update_rate(void);
    
void get_gyro_raw(float *gyro);
void get_gyro_lpf(float *gyro);
float get_gyro_update_rate(void);

#endif
