#ifndef __IMU_H__
#define __IMU_H__

#include "debug_link.h"

void imu_init(void);
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

float get_imu_temperature(void);

#endif
