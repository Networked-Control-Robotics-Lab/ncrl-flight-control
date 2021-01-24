#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "proj_config.h"
#include "ist8310.h"
#include "optitrack.h"
#include "debug_link.h"
#include "ahrs_selector.h"
#include "imu.h"

/* debug variables */
float debug_mag_vec_angle_change_rate;
float debug_gyro_angular_velocity;

void get_compass_raw(float *mag)
{
	ist8310_get_mag_raw(mag);
}

void get_compass_lpf(float *mag)
{
	ist8310_get_mag_lpf(mag);
}

bool is_compass_available(void)
{
#if (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_COMPASS)
	return ist8310_available();
#elif (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_OPTITRACK)
	return optitrack_available();
#else
	return false;
#endif
}

bool is_compass_quality_good(float *mag_now, float *mag_last)
{
	//TODO: check magnetic field size (normally about 25 to 65 uT)

	/* no data */
	if(mag_now[0] == 0.0f && mag_now[1] == 0.0f && mag_now[2] == 0.0f) {
		return false;
	}

	/* calculate angle change rate from compass */
	float mag_update_period = 0.02; //XXX
	float mag_vec_angle_change = calc_vectors_angle_3x1(mag_last, mag_now);
	float mag_vec_angle_change_rate = mag_vec_angle_change / mag_update_period;

	/* get angular velocity from gyroscope */
	float gyro[3], gyro_magnitude;
	get_gyro_lpf(gyro);
	norm_3x1(gyro, &gyro_magnitude);

	/* debugging: */
	debug_mag_vec_angle_change_rate = mag_vec_angle_change_rate;
	debug_gyro_angular_velocity = gyro_magnitude;

	/* if the angle rate difference of compass and gyroscope is larger than 90deg/sec
	 * then the compass quality is highly possible to be bad */
	if(fabs(mag_vec_angle_change_rate - gyro_magnitude) > 90.0f) {
		return false;
	}

	/* compass quality is good */
	return true;
}

float get_compass_update_rate(void)
{
	return ist8310_get_update_rate();
}

float get_compass_raw_strength(void)
{
	return ist8310_get_mag_raw_strength();
}

float get_compass_lpf_strength(void)
{
	return ist8310_get_mag_lpf_strength();
}

void compass_undistortion(float *mag)
{
	ist8310_undistortion(mag);
}

void send_compass_debug_message(debug_msg_t *payload)
{
	float mag_raw[3] = {0.0f};
	get_compass_lpf(mag_raw);

	float mag_strength = get_compass_lpf_strength();
	float update_freq = get_compass_update_rate();

	pack_debug_debug_message_header(payload, MESSAGE_ID_COMPASS);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&mag_strength, payload);
	pack_debug_debug_message_float(&update_freq, payload);
	pack_debug_debug_message_float(&debug_gyro_angular_velocity, payload);
	pack_debug_debug_message_float(&debug_mag_vec_angle_change_rate, payload);
}
