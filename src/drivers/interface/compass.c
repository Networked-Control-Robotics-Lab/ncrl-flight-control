#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "proj_config.h"
#include "ist8310.h"
#include "optitrack.h"
#include "debug_link.h"
#include "ahrs_selector.h"

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

	/* get current ahrs yaw angle */
	float curr_ahrs_roll, curr_ahrs_pitch, curr_ahrs_yaw;
	get_attitude_euler_angles(&curr_ahrs_roll, &curr_ahrs_pitch, &curr_ahrs_yaw);

	/* get compass yaw angle */
	float compass_yaw = -rad_to_deg(atan2(mag_raw[1], mag_raw[0]));

	pack_debug_debug_message_header(payload, MESSAGE_ID_COMPASS);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&mag_strength, payload);
	pack_debug_debug_message_float(&update_freq, payload);
	pack_debug_debug_message_float(&curr_ahrs_yaw, payload);
	pack_debug_debug_message_float(&compass_yaw, payload);
}
