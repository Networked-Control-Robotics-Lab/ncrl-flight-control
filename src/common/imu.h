#ifndef __IMU_H__
#define __IMU_H__

#include "vector.h"

typedef struct {
	vector3d_16_t accel_unscaled;
	vector3d_16_t gyro_unscaled;
	int16_t temp_unscaled;

	vector3d_f_t accel_raw;
	vector3d_f_t gyro_raw;
	vector3d_f_t mag_raw;
	float temp;

	vector3d_f_t accel_lpf;
	vector3d_f_t gyro_lpf;
	vector3d_f_t mag_lpf;
} imu_t;

#endif
