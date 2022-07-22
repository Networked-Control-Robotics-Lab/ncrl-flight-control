#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "proj_config.h"
#include "ist8310.h"
#include "optitrack.h"
#include "vins_mono.h"
#include "debug_link.h"
#include "ahrs.h"
#include "imu.h"
#include "proj_config.h"

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
	return ist8310_available();
}

void compass_wait_until_stable(void)
{
#if (ENABLE_MAGNETOMETER != 0)
	ist8310_wait_until_stable();
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
}
