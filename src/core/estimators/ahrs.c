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

extern optitrack_t optitrack;

madgwick_t madgwick_ahrs;

void ahrs_init(void)
{
	complementary_ahrs_init(0.0025);

	madgwick_init(&madgwick_ahrs, 400, 0.3);
}

void ahrs_gyro_integration(float *q, float *gyro, float time)
{
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[1];
	w[2] = gyro[2];
	w[3] = gyro[3];

	float q_dot[4];
	quaternion_mult(q, w, q_dot);

	q[0] += q_dot[0] * 0.5 * time;
	q[1] += q_dot[1] * 0.5 * time;
	q[2] += q_dot[2] * 0.5 * time;
	q[3] += q_dot[3] * 0.5 * time;
	quat_normalize(q);
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

void ahrs_estimate(ahrs_t *ahrs, float *accel, float *gyro)
{
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

#if (SELECT_AHRS == AHRS_COMPLEMENTARY_FILTER)
	ahrs_complementary_filter_estimate(ahrs->q, gravity, gyro_rad);
#elif (SELECT_AHRS == AHRS_MADGWICK_FILTER)
	madgwick_imu_ahrs(&madgwick_ahrs, gravity, gyro_rad);
	quaternion_copy(ahrs->q, madgwick_ahrs.q);
#endif

#if (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
	reset_quaternion_yaw_angle(ahrs->q);
	align_ahrs_with_optitrack_yaw(ahrs->q);
#endif

	euler_t euler;
	quat_to_euler(ahrs->q, &euler);
	ahrs->attitude.roll = rad_to_deg(euler.roll);
	ahrs->attitude.pitch = rad_to_deg(euler.pitch);
	ahrs->attitude.yaw = rad_to_deg(euler.yaw);
}
