#ifndef __IMU_H__
#define __IMU_H__

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

bool is_compass_present(void);

void get_imu_accel_raw(float *accel_raw);
void get_imu_gyro_raw(float *gyro_raw);
void get_imu_compass_raw(float *mag_raw);

#endif