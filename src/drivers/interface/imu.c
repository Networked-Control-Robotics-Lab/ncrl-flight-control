#include <stdint.h>
#include <stdbool.h>
#include "proj_config.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "debug_link.h"

void imu_init(void)
{
	mpu6500_init();
}

bool imu_calibration_not_finished(void)
{
	return mpu6500_calibration_not_finished();
}

void reset_accel_scale_factor(void)
{
	mpu6500_reset_scale_factor();
}

void reset_accel_bias(void)
{
	mpu6500_reset_bias();
}

void set_accel_scale_factor(float x_scale, float y_scale, float z_scale)
{
	mpu6500_set_scale_factor(x_scale, y_scale, z_scale);
}

void set_accel_bias(float x_offset, float y_offset, float z_offset)
{
	mpu6500_set_bias(x_offset, y_offset, z_offset);
}

void get_accel_raw(float *accel)
{
	mpu6500_get_raw_accel(accel);
}

void get_accel_lpf(float *accel)
{
	mpu6500_get_filtered_accel(accel);
}

void get_gyro_raw(float *gyro)
{
	mpu6500_get_gyro_raw(gyro);
}

void get_gyro_lpf(float *gyro)
{
	mpu6500_get_gyro_lpf(gyro);
}

float get_accel_update_rate(void)
{
	return 0.0f;
}

float get_gyro_update_rate(void)
{
	return 0.0f;
}

float get_imu_temperature(void)
{
	return mpu6500_get_temperature();

}
