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
} euler_t;

typedef struct {
	euler_t attitude;
	float q[4];
} ahrs_t;

void imu_read(vector3d_f_t *accel, vector3d_f_t *gyro);

void ahrs_init(vector3d_f_t *init_accel);
void ahrs_estimate(euler_t *att_euler, float *att_quat,vector3d_f_t *accel, vector3d_f_t *gyro);

void quat_normalize(float *q);

void euler_to_quat(euler_t *euler, float *q);
void quat_to_euler(float *q, euler_t *euler);

void calc_attitude_use_accel(euler_t *att_estimated, vector3d_f_t *accel);

#endif
