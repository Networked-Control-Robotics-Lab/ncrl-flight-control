#include <stdint.h>
#include <stdbool.h>
#include "proj_config.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "debug_link.h"

void get_imu_accel_raw(float *accel_raw)
{
}

void get_imu_gyro_raw(float *gyro_raw)
{
}

void get_imu_compass_raw(float *mag_raw)
{
	ist8310_get_raw_mag(mag_raw);
}

bool is_compass_present(void)
{
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG)
	return true;
#else
	return false;
#endif
}

float get_imu_accel_update_freq(void)
{
	return 0.0f;
}

float get_imu_gyro_update_freq(void)
{
	return 0.0f;
}

float get_imu_compass_update_freq(void)
{
	return ist8310_get_update_freq();
}

void send_compass_debug_message(debug_msg_t *payload)
{
	float mag_raw[3] = {0.0f};
	get_imu_compass_raw(mag_raw);

	pack_debug_debug_message_header(payload, MESSAGE_ID_COMPASS);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
}
