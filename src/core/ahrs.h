#ifndef __AHRS__
#define __AHRS__

#include "vector.h"

#define AHRS_SELECT_EKF 0
#define AHRS_SELECT_CF 1

#define deg_to_rad(angle) (angle * 0.01745329252)
#define rad_to_deg(radian) (radian * 57.2957795056)

typedef struct {
	vector3d_16_t unscaled_accel;
	vector3d_16_t unscaled_gyro;
	vector3d_16_t unscaled_mag;
	int16_t unscaled_temp;

	vector3d_f_t raw_accel;
	vector3d_f_t raw_gyro;
	vector3d_f_t raw_mag;

	vector3d_f_t filtered_accel;
	vector3d_f_t filtered_gyro;
	vector3d_f_t filtered_mag;
} imu_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} attitude_t;

typedef struct {
	attitude_t attitude;
} ahrs_t;

void ahrs_init(void);
void ahrs_estimate_euler(float *roll, float *pitch, float *yaw);


void quat_normalize(float *q);

void euler_to_quat(attitude_t *euler, float *q);
void quat_to_euler(float *q, attitude_t *euler);

void calc_attitude_use_accel(void);

#endif
