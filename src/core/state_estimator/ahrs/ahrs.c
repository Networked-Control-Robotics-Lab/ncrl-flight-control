#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "mpu6500.h"
#include "optitrack.h"
#include "ahrs.h"
#include "comp_ahrs.h"
#include "madgwick_ahrs.h"
#include "eskf_ahrs.h"
#include "lpf.h"
#include "uart.h"
#include "matrix.h"
#include "delay.h"
#include "proj_config.h"
#include "se3_math.h"
#include "quaternion.h"
#include "free_fall.h"
#include "compass.h"
#include "ins_sensor_sync.h"
#include "debug_link.h"
#include "sys_time.h"
#include "led.h"

extern optitrack_t optitrack;
extern SemaphoreHandle_t flight_ctrl_semphr;

attitude_t attitude;

madgwick_t madgwick_ahrs;

bool use_compass;
volatile bool mag_error;

/* debugging */
struct {
	float good;
	float compass_yaw;
	float ahrs_yaw;
} compass_quality_debug;

void ahrs_init(void)
{
	complementary_ahrs_init(0.0025);

	madgwick_init(&madgwick_ahrs, 400, 0.13);

	eskf_ahrs_init(0.0025);

	switch(SELECT_HEADING_SENSOR) {
	case HEADING_SENSOR_USE_COMPASS:
		use_compass = true;
		break;
	case HEADING_SENSOR_USE_OPTITRACK:
	default:
		use_compass = false;
	}
}

void init_ahrs_quaternion_with_accel_and_compass(float *q_ahrs)
{
	/* compass not available */
	if(is_compass_available() == false) {
		q_ahrs[0] = 1.0f;
		q_ahrs[1] = 0.0f;
		q_ahrs[2] = 0.0f;
		q_ahrs[3] = 0.0f;
		return;
	}

	/* initialize quaternion with compass and accelerometer */
	float accel[3];
	get_accel_lpf(accel);
	accel[0] *= -1;
	accel[1] *= -1;
	accel[2] *= -1;
	normalize_3x1(accel);

	float mag[3];
	get_compass_lpf(mag);
	normalize_3x1(mag);

	float q_mag[4];
	convert_magnetic_field_to_quat(mag, q_mag);

	float q_accel[4];
	convert_gravity_to_quat(accel, q_accel);

	quaternion_mult(q_accel, q_mag, q_ahrs);
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

bool ahrs_compass_quality_test(float *mag_new)
{
	//once the compass is detected as unstable, this flag will be trigger on for 1 seconds
	static bool compass_is_stable = true;
	static float last_failed_time = 0;

	if(compass_is_stable == false) {
		if((get_sys_time_s() - last_failed_time) > 0.5f) {
			compass_is_stable = true;
		}
	}

	//TODO: check magnetic field size (normally about 25 to 65 uT)

	/* no data */
	if(mag_new[0] == 0.0f && mag_new[1] == 0.0f && mag_new[2] == 0.0f) {
		last_failed_time = get_sys_time_s();
		compass_is_stable = false;
	}

	float compass_ahrs_yaw_diff;

	float roll, pitch, yaw;
	get_attitude_euler_angles(&roll, &pitch, &yaw);

#if 0
	/*=============================================================*
	 * euler angles based ahrs-compass angle difference comparison *
	 *=============================================================*/
	float compass_angle = rad_to_deg(-atan2f(mag_new[1], mag_new[0]));

	if(compass_angle < 0 && yaw > 0) {
		compass_ahrs_yaw_diff = fabs(compass_angle + yaw);
	} else if(compass_angle > 0 && yaw < 0) {
		compass_ahrs_yaw_diff = fabs(compass_angle + yaw);
	} else {
		compass_ahrs_yaw_diff = fabs(compass_angle - yaw);
	}

	if(compass_ahrs_yaw_diff > 45) {
		last_failed_time = get_sys_time_s();
		compass_is_stable = false;
	}

	/* debugging */
	compass_quality_debug.good = (float)compass_is_stable;
	compass_quality_debug.compass_yaw = compass_angle;
	compass_quality_debug.ahrs_yaw = yaw;
#else
	/*===========================================================*
	 * quaternion based ahrs-compass angle difference comparison *
	 *===========================================================*/
	float mag_normalized[3];
	mag_normalized[0] = mag_new[0];
	mag_normalized[1] = mag_new[1];
	mag_normalized[2] = mag_new[2];
	normalize_3x1(mag_normalized);

	//get rotation matrix of current attitude
	float *R_b2i;
	get_rotation_matrix_b2i(&R_b2i);

	//calculate predicted earth frame magnetic vector
	float l_predict[3], q_delta_mag[4];
	l_predict[0] = R_b2i[0*3+0]*mag_normalized[0] + R_b2i[0*3+1]*mag_normalized[1] + R_b2i[0*3+2]*mag_normalized[2];
	l_predict[1] = R_b2i[1*3+0]*mag_normalized[0] + R_b2i[1*3+1]*mag_normalized[1] + R_b2i[1*3+2]*mag_normalized[2];
	l_predict[2] = R_b2i[2*3+0]*mag_normalized[0] + R_b2i[2*3+1]*mag_normalized[1] + R_b2i[2*3+2]*mag_normalized[2];

	//calculate delta quaternion of compass correction
	convert_magnetic_field_to_quat(l_predict, q_delta_mag);

	//get attitude quaternion from ahrs
	float q_ahrs_b2i[4], q_ahrs_i2b[4];
	get_attitude_quaternion(q_ahrs_i2b);
	quaternion_conj(q_ahrs_i2b, q_ahrs_b2i);

	//predict new attitude quaternion with 100% trust of compass
	float q_mag_i2b[4], q_mag_b2i[4];
	quaternion_mult(q_ahrs_i2b, q_delta_mag, q_mag_i2b);
	quaternion_conj(q_mag_i2b, q_mag_b2i);

	//calculate rotation difference between q_ahrs and q_mag
	float q_diff[4];
	quaternion_mult(q_ahrs_i2b, q_mag_b2i, q_diff);

	//get euler principal axis agnle of q_mag minus q_ahrs
	compass_ahrs_yaw_diff = rad_to_deg(acos(q_diff[0]));

	if(compass_ahrs_yaw_diff > 45) {
		last_failed_time = get_sys_time_s();
		compass_is_stable = false;
	}

	/* debugging */
	compass_quality_debug.good = (float)compass_is_stable;

	float q0 = q_mag_i2b[0];
	float q1 = q_mag_i2b[1];
	float q2 = q_mag_i2b[2];
	float q3 = q_mag_i2b[3];
	compass_quality_debug.compass_yaw = rad_to_deg(atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3)));

	compass_quality_debug.ahrs_yaw = yaw;
#endif

	/* change led indicator to yellow if sensor fault detected */
	set_rgb_led_service_sensor_error_flag(!compass_is_stable);

	return compass_is_stable;
}

void ahrs_estimate(void)
{
	static bool compass_init = false;

	float accel[3];
	float gyro[3];
	float mag[3];

	/* read imu data (update with 1KHz, read with 400Hz) */
	get_accel_lpf(accel);
	get_gyro_lpf(gyro);

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

	/* check compass data is availabe or not */
	bool recvd_compass = ins_compass_sync_buffer_available();
	if(recvd_compass == true) {
		/* pop compass data from ins sync buffer (update and read with 50Hz) */
		ins_compass_sync_buffer_pop(mag);

		if(compass_init == false) {
			float mag_raw[3];
			get_compass_raw(mag_raw);
			//convert_magnetic_field_to_quat(mag_raw, attitude.q);
			compass_init = true;
		}

		/* check compass quality */
		if(ahrs_compass_quality_test(mag) == true) {
			//good quality, apply calibration
			compass_undistortion(mag);
			mag_error = false;
		} else {
			mag_error = true;
		}
	}

#if (SELECT_AHRS == AHRS_COMPLEMENTARY_FILTER)
	if(mag_error == false && recvd_compass == true) {
		ahrs_marg_complementary_filter_estimate(attitude.q, gravity, gyro_rad, mag);
	} else {
		ahrs_imu_complementary_filter_estimate(attitude.q, gravity, gyro_rad);
	}
#elif (SELECT_AHRS == AHRS_MADGWICK_FILTER)
	if(mag_error == false && recvd_compass == true) {
		madgwick_margs_ahrs(&madgwick_ahrs, gravity, gyro_rad, mag);
	} else {
		madgwick_imu_ahrs(&madgwick_ahrs, gravity, gyro_rad);
	}
	quaternion_copy(attitude.q, madgwick_ahrs.q);
#elif (SELECT_AHRS == AHRS_ESKF)
	eskf_ahrs_predict(gyro_rad);
	eskf_ahrs_accelerometer_correct(gravity);

	if(mag_error == false && recvd_compass == true) {
		eskf_ahrs_magnetometer_correct(mag);
	}

	get_eskf_attitude_quaternion(attitude.q);
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

	quat_to_rotation_matrix(attitude.q, attitude.R_b2i, attitude.R_i2b);
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

void get_rotation_matrix_b2i(float **R_b2i)
{
	*R_b2i = attitude.R_b2i;
}

void get_rotation_matrix_i2b(float **R_i2b)
{
	*R_i2b = attitude.R_i2b;
}

void send_ahrs_compass_quality_check_debug_message(debug_msg_t *payload)
{
	float mag_raw[3] = {0.0f};
	get_compass_lpf(mag_raw);

	float mag_strength = get_compass_lpf_strength();
	float update_freq = get_compass_update_rate();

	pack_debug_debug_message_header(payload, MESSAGE_ID_AHRS_COMPASS_QUALITY_CHECK);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&mag_strength, payload);
	pack_debug_debug_message_float(&update_freq, payload);
	pack_debug_debug_message_float(&compass_quality_debug.good, payload);
	pack_debug_debug_message_float(&compass_quality_debug.compass_yaw, payload);
	pack_debug_debug_message_float(&compass_quality_debug.ahrs_yaw, payload);
}
