#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "mpu6500.h"
#include "optitrack.h"
#include "vins_mono.h"
#include "ahrs.h"
#include "ahrs_comp_filter.h"
#include "ahrs_madgwick.h"
#include "ahrs_eskf.h"
#include "ahrs_optitrack.h"
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
#include "system_state.h"
#include "vio.h"

extern SemaphoreHandle_t flight_ctrl_semphr;

madgwick_t madgwick_ahrs;

bool use_compass;
bool compass_init;
volatile bool mag_error;

/* debugging */
struct {
	float good;
	float compass_yaw_rate;
	float ahrs_yaw_rate;
} compass_quality_debug;

void ahrs_init(void)
{
	complementary_ahrs_init(0.0025);
	madgwick_init(&madgwick_ahrs, 400, 0.13);
	eskf_ahrs_init(0.0025);
	optitrack_ahrs_init(0.0025);

	switch(SELECT_HEADING_SENSOR) {
	case HEADING_FUSION_USE_COMPASS:
		use_compass = true;
		break;
	case HEADING_FUSION_USE_OPTITRACK:
	case HEADING_FUSION_USE_VINS_MONO:
	default:
		use_compass = false;
	}

	compass_init = false;
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

void realign_ahrs_yaw_direction(float *q_ahrs, float *q_align_reference)
{
	/* reset yaw angle of the ahrs quaternion */
	reset_quaternion_yaw_angle(q_ahrs);

	/* realign ahrs quaternion with a accurate reference angle */
	euler_t reference_euler;
	quat_to_euler(q_align_reference, &reference_euler);

	float half_psi = reference_euler.yaw * 0.5f;

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

//old code: https://gist.github.com/shengwen-tw/b096963d11739209bed8556e6ed3f5ba
bool ahrs_compass_quality_test(float *mag_new)
{
	float mag_b[3] = {mag_new[0], mag_new[1], mag_new[2]};
	normalize_3x1(mag_b);

	static bool initialized = false;
	static bool compass_unstable = false; //long term unstable flag
	static float statistic_time_last = 0;
	static float led_time_last = 0;
	static float compass_yaw_last = 0;
	static int failed_times = 0;

	/* initialization */
	if(initialized == false) {
		statistic_time_last = get_sys_time_s();
		compass_yaw_last = rad_to_deg(-atan2f(mag_b[1], mag_b[0]));
		initialized = true;
		return false;
	}

	/*==================================*
	 * calculate yaw rate from the ahrs *
	 *==================================*/

	float gyro[3];
	get_gyro_raw(gyro);
	//gyro[0] = deg_to_rad(gyro[0]);
	gyro[1] = deg_to_rad(gyro[1]);
	gyro[2] = deg_to_rad(gyro[2]);

	float roll, pitch, yaw;
	get_attitude_euler_angles(&roll, &pitch, &yaw);

	float cos_phi = arm_cos_f32(deg_to_rad(roll));
	float cos_theta = arm_cos_f32(deg_to_rad(pitch));
	float sin_phi = arm_sin_f32(deg_to_rad(roll));
	float sin_theta = arm_sin_f32(deg_to_rad(pitch));

	/* calculate the ahrs yaw rate from angular velocity */
	float ahrs_yaw_rate = rad_to_deg((sin_phi / cos_theta) * gyro[1] +
	                                 (cos_phi / cos_theta) * gyro[2]);

	/*=====================================*
	 * calculate yaw rate from the compass *
	 *=====================================*/

	/* coordinate transform */
	float R_b2i[3*3]; //let psi = 0
	R_b2i[0*3 + 0] = cos_theta;
	R_b2i[0*3 + 1] = (sin_phi * sin_theta);
	R_b2i[0*3 + 2] = (cos_phi * sin_theta);
	R_b2i[1*3 + 0] = 0.0f;
	R_b2i[1*3 + 1] = cos_phi;
	R_b2i[1*3 + 2] = -sin_phi;
	R_b2i[2*3 + 0] = -sin_theta;
	R_b2i[2*3 + 1] = sin_phi * cos_theta;
	R_b2i[2*3 + 2] = cos_phi * cos_theta;

	float mag_i[3];
	calc_matrix_multiply_vector_3d(mag_i, mag_b, R_b2i);

	/* calculate the compass yaw angle */
	float compass_yaw = rad_to_deg(-atan2f(mag_i[1], mag_i[0]));

	/* calculate the compass yaw rate */
	float compass_yaw_rate;
	float compass_freq = get_compass_update_rate();

	/* the yaw angle is not continous at +-180 degree */
	if(compass_yaw > 90.0f && compass_yaw_last < -90.0f) {
		compass_yaw_rate = (-360.0f - compass_yaw_last + compass_yaw) * compass_freq;
	} else if(compass_yaw < -90.0f && compass_yaw_last > 90.0f) {
		compass_yaw_rate = (360.0f - compass_yaw_last + compass_yaw) * compass_freq;
	} else {
		compass_yaw_rate = (compass_yaw - compass_yaw_last) * compass_freq;
	}

	/* compare the ahrs yaw rate with the compass */
	float compass_gyro_yaw_rate_diff = fabs(ahrs_yaw_rate - compass_yaw_rate);

	bool usable_measurement = true;

	if(compass_gyro_yaw_rate_diff > 200) {
		/* large deviation, the compass is not stable */
		usable_measurement = false;
		failed_times++;
	}

	compass_yaw_last = compass_yaw;

	/*===============================================================*
	 * failure time counting:                                        *
	 * if 1/4 of the measurements was detected unusable in a second, *
	 * the compass is considered to be unstable                      *
	 *===============================================================*/

	float curr_time = get_sys_time_s();
	float elapsed_time;

	if(failed_times > ((int)compass_freq / 4)) {
		compass_unstable = true;
		led_time_last = curr_time;
	} else {
		/* once the compass_unstable flag is set, it takes at least
		 * 3 seconds to reset */
		elapsed_time = curr_time - led_time_last;
		if(elapsed_time > 3) {
			compass_unstable = false;
		}
	}

	/* reset the failed times counter every seconds */
	elapsed_time = curr_time - statistic_time_last;
	if(elapsed_time > 1) {
		statistic_time_last = curr_time;
		failed_times = 0;
	}

	/* change led indicator to pink if sensor_unstable flag is set*/
	set_rgb_led_service_sensor_error_flag(compass_unstable);

	/* debugging */
	compass_quality_debug.good = (float)(usable_measurement);
	compass_quality_debug.compass_yaw_rate = compass_yaw_rate;
	compass_quality_debug.ahrs_yaw_rate = ahrs_yaw_rate;

	return usable_measurement;
}

void ahrs_complementary_filter_estimate(float *q, float *gravity, float *gyro_rad,
                                        float *mag, bool fuse_mag)
{
	if(fuse_mag == true) {
		/* fuse gyroscope, accelerometer and magnetometer */
		ahrs_marg_complementary_filter_estimate(q, gravity, gyro_rad, mag);
	} else {
		/* fuse gyroscope and accelerometer */
		ahrs_imu_complementary_filter_estimate(q, gravity, gyro_rad);
	}
}

void ahrs_madgwick_filter_estimate(float *q, float *gravity, float *gyro_rad,
                                   float *mag, bool fuse_mag)
{
	if(fuse_mag == true) {
		/* fuse gyroscope, accelerometer and magnetometer */
		madgwick_margs_ahrs(&madgwick_ahrs, gravity, gyro_rad, mag);
	} else {
		/* fuse gyroscope and accelerometer */
		madgwick_imu_ahrs(&madgwick_ahrs, gravity, gyro_rad);
	}
	quaternion_copy(q, madgwick_ahrs.q);
}

void ahrs_eskf_estimate(float *q, float *gravity, float *gyro_rad,
                        float *mag, bool fuse_mag)
{
	/* update */
	eskf_ahrs_predict(gyro_rad);

	/* accelerometer correction */
	eskf_ahrs_accelerometer_correct(gravity);

	/* magnetometer correction */
	if(fuse_mag == true) {
		eskf_ahrs_magnetometer_correct(mag);
	}

	/* retrieve estimation result from eskf */
	get_eskf_attitude_quaternion(q);
}

void ahrs_optitrack_estimate(float *q, float *gravity, float *gyro_rad,
                             float *mag, bool fuse_mag)
{
	/* fuse optitrack and gyroscope */
	ahrs_optitrack_imu_fuse_estimate(q, gyro_rad);
}

void ahrs_estimate(attitude_t *attitude)
{
	float accel[3];
	float gyro[3];
	float mag[3];

	/* read imu data (update with 1KHz, read with 400Hz) */
	get_accel_lpf(accel);
	get_gyro_lpf(gyro);

	/* ----------------------------------------------------- *
	 * accelerometer sensor model:                           *
	 * a_imu = (R_i2b * a_translation) - (R_i2b * g)         *
	 * ----------------------------------------------------- *
	 * assuming a_translation = 0, measurment of the gravity *
	 * in the body-fixed frame can be obtained from:         *
	 * a_gravity = -a_imu = (R_i2b * g)                      *
	 * ----------------------------------------------------- */
	float gravity[3];
	gravity[0] = -accel[0];
	gravity[1] = -accel[1];
	gravity[2] = -accel[2];

	/* convert gyroscope measurement unit from [deg/s] to [rad/s] */
	float gyro_rad[3];
	gyro_rad[0] = deg_to_rad(gyro[0]);
	gyro_rad[1] = deg_to_rad(gyro[1]);
	gyro_rad[2] = deg_to_rad(gyro[2]);

	/* check compass data is availabe or not */
	bool recvd_compass = ins_compass_sync_buffer_available();
	if(recvd_compass == true) {
		/* pop compass data from ins sync buffer */
		ins_compass_sync_buffer_pop(mag);

		/* compass quality checker initialization */
		if(compass_init == false) {
			float mag_raw[3];
			get_compass_raw(mag_raw);
			compass_init = true;
		}

		/* check compass quality */
		if(ahrs_compass_quality_test(mag) == true) {
			/* compass is good */
			mag_error = false;
		} else {
			/* error detected */
			mag_error = true;
		}
	}

	/* magnetometer fusion is enabled only if new measurement is received,
	 * no error detected and heading sensor soruce is set as compass */
	bool fuse_mag = (mag_error == false) && (recvd_compass == true) &&
	                (get_heading_sensor() == HEADING_FUSION_USE_COMPASS);

#if (SELECT_AHRS == AHRS_COMPLEMENTARY_FILTER)
	ahrs_complementary_filter_estimate(attitude->q, gravity, gyro_rad, mag, fuse_mag);
#elif (SELECT_AHRS == AHRS_MADGWICK_FILTER)
	ahrs_madgwick_filter_estimate(attitude->q, gravity, gyro_rad, mag, fuse_mag);
#elif (SELECT_AHRS == AHRS_ESKF)
	ahrs_eskf_estimate(attitude->q, gravity, gyro_rad, mag, fuse_mag);
#elif (SELECT_AHRS == AHRS_OPTITRACK)
	ahrs_optitrack_estimate(attitude->q, gravity, gyro_rad, mag, fuse_mag);
#endif

	/* heading realignment */
	switch(get_heading_sensor()) {
	case HEADING_FUSION_USE_OPTITRACK: {
		float q_optitrack[4];
		optitrack_get_quaternion(q_optitrack);

		if(optitrack_available() == true) {
			realign_ahrs_yaw_direction(attitude->q, q_optitrack);
		}
		break;
	}
	case HEADING_FUSION_USE_VINS_MONO: {
		float q_vio[4];
		vio_get_quaternion(q_vio);

		if(vio_available() == true) {
			realign_ahrs_yaw_direction(attitude->q, q_vio);
		}
		break;
	}
	}

	/* convert quaternion to euler angles */
	euler_t euler;
	quat_to_euler(attitude->q, &euler);
	attitude->roll = rad_to_deg(euler.roll);
	attitude->pitch = rad_to_deg(euler.pitch);
	attitude->yaw = rad_to_deg(euler.yaw);

	/* convert quaternion to rotation matrix */
	quat_to_rotation_matrix(attitude->q, attitude->R_b2i, attitude->R_i2b);
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
	pack_debug_debug_message_float(&compass_quality_debug.compass_yaw_rate, payload);
	pack_debug_debug_message_float(&compass_quality_debug.ahrs_yaw_rate, payload);
}
