#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "mpu6500.h"
#include "optitrack.h"
#include "ahrs.h"
#include "comp_ahrs.h"
#include "madgwick_ahrs.h"
#include "lpf.h"
#include "uart.h"
#include "matrix.h"
#include "delay.h"
#include "proj_config.h"
#include "se3_math.h"
#include "quaternion.h"
#include "free_fall.h"
#include "compass.h"

extern optitrack_t optitrack;

attitude_t attitude;

madgwick_t madgwick_ahrs;

bool use_compass;
volatile bool mag_error;

void ahrs_init(void)
{
	complementary_ahrs_init(0.0025);

	madgwick_init(&madgwick_ahrs, 400, 0.13);

	switch(SELECT_HEADING_SENSOR) {
	case HEADING_SENSOR_USE_COMPASS:
		use_compass = true;
		break;
	case HEADING_SENSOR_USE_OPTITRACK:
	default:
		use_compass = false;
	}
}

void ahrs_gyro_integration(float *q, float *gyro, float time)
{
#if 0
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[0];
	w[2] = gyro[1];
	w[3] = gyro[2];

	float q_dot[4];
	quaternion_mult(q, w, q_dot);
	q[0] += q_dot[0] * 0.5 * time;
	q[1] += q_dot[1] * 0.5 * time;
	q[2] += q_dot[2] * 0.5 * time;
	q[3] += q_dot[3] * 0.5 * time;
#else
	float w[3];
	w[0] = gyro[0] * time;
	w[1] = gyro[1] * time;
	w[2] = gyro[2] * time;

	float w_norm;
	arm_sqrt_f32((w[0]*w[0]) + (w[1]*w[1]) + (w[2]*w[2]), &w_norm);

	/* reciprocal of w_norm is unstable when w_norm is ~0 */
	if(w_norm > 1.0e-12) {
		float q_last[4];
		quaternion_copy(q_last, q);

		float half_w_norm = 0.5f * w_norm;
		float recip_w_norm = 1.0f / w_norm;

		float q_rot[4];
		q_rot[0] = arm_cos_f32(half_w_norm);
		q_rot[1] = arm_sin_f32(half_w_norm) * w[0] * recip_w_norm;
		q_rot[2] = arm_sin_f32(half_w_norm) * w[1] * recip_w_norm;
		q_rot[3] = arm_sin_f32(half_w_norm) * w[2] * recip_w_norm;

		quaternion_mult(q_last, q_rot, q);
		quat_normalize(q);
	}
#endif
}

void align_ahrs_with_optitrack_yaw(float *q_ahrs)
{
	if(optitrack_available() == true) {
		euler_t optitrack_euler;
		quat_to_euler(optitrack.q, &optitrack_euler);

		float half_psi = optitrack_euler.yaw * 0.5f;

		float q_yaw[4];
		q_yaw[0] = arm_cos_f32(half_psi);
		q_yaw[1] = 0.0f;
		q_yaw[2] = 0.0f;
		q_yaw[3] = arm_sin_f32(half_psi);

		float q_original[4];
		q_original[0] = q_ahrs[0];
		q_original[1] = q_ahrs[1];
		q_original[2] = q_ahrs[2];
		q_original[3] = q_ahrs[3];
		quaternion_mult(q_yaw, q_original, q_ahrs);
	}
}

void reset_quaternion_yaw_angle(float *q)
{
	float q_original[4];
	q_original[0] = q[0];
	q_original[1] = q[1];
	q_original[2] = q[2];
	q_original[3] = q[3];

	euler_t euler;
	quat_to_euler(q, &euler);
	float half_psi = euler.yaw / 2.0f;

	float q_negative_yaw[4];
	q_negative_yaw[0] = arm_cos_f32(-half_psi);
	q_negative_yaw[1] = 0.0f;
	q_negative_yaw[2] = 0.0f;
	q_negative_yaw[3] = arm_sin_f32(-half_psi);

	quaternion_mult(q_negative_yaw, q_original, q);
}

bool compass_quality_is_good(float *mag)
{
	/* TODO: check magnetic field size (normally about 25 to 65 uT) */

	/* no data output, bad connection and so forth */
	if(mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f) {
		return false;
	} else {
		return true;
	}
}

void ahrs_estimate(void)
{
	float accel[3];
	float gyro[3];
	float mag[3];
	get_accel_lpf(accel);
	get_gyro_lpf(gyro);
	get_compass_raw(mag);

	/* note that acceleromter senses the negative gravity acceleration (normal force)
	 * a_imu = (R(phi, theta, psi) * a_translation) - (R(phi, theta, psi) * g) */
	float gravity[3];
	gravity[0] = -accel[0];
	gravity[1] = -accel[1];
	gravity[2] = -accel[2];

	float gyro_rad[3];
	gyro_rad[0] = deg_to_rad(gyro[0]);
	gyro_rad[1] = deg_to_rad(gyro[1]);
	gyro_rad[2] = deg_to_rad(gyro[2]);

	if(compass_quality_is_good(mag) == true) {
		compass_undistortion(mag);
		mag_error = false;
	} else {
		mag_error = true;
	}

#if (SELECT_AHRS == AHRS_COMPLEMENTARY_FILTER)
	if(mag_error == false && use_compass == true) {
		ahrs_marg_complementary_filter_estimate(attitude.q, gravity, gyro_rad, mag);
	} else {
		ahrs_imu_complementary_filter_estimate(attitude.q, gravity, gyro_rad);
	}
#elif (SELECT_AHRS == AHRS_MADGWICK_FILTER)
	if(mag_error == false && use_compass == true) {
		madgwick_margs_ahrs(&madgwick_ahrs, gravity, gyro_rad, mag);
	} else {
		madgwick_imu_ahrs(&madgwick_ahrs, gravity, gyro_rad);
	}
	quaternion_copy(attitude.q, madgwick_ahrs.q);
#endif

#if (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_OPTITRACK)
	reset_quaternion_yaw_angle(attitude.q);
	align_ahrs_with_optitrack_yaw(attitude.q);
#endif

	euler_t euler;
	quat_to_euler(attitude.q, &euler);
	attitude.roll = rad_to_deg(euler.roll);
	attitude.pitch = rad_to_deg(euler.pitch);
	attitude.yaw = rad_to_deg(euler.yaw);

	quat_to_rotation_matrix(attitude.q, attitude.rotation_mat_data,
	                        attitude.transposed_rotation_mat_data);
}

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw)
{
	*roll = attitude.roll;
	*pitch = attitude.pitch;
	*yaw = attitude.yaw;
}

void get_attitude_quaternion(float *q)
{
	q[0] = attitude.q[0];
	q[1] = attitude.q[1];
	q[2] = attitude.q[2];
	q[3] = attitude.q[3];
}

void get_attitude_direction_cosine_matrix(float **rotation_mat_ptr)
{
	*rotation_mat_ptr = attitude.rotation_mat_data;
}

void get_attitude_transposed_direction_cosine_matrix(float **transposed_rotation_mat_ptr)
{
	*transposed_rotation_mat_ptr = attitude.transposed_rotation_mat_data;
}
