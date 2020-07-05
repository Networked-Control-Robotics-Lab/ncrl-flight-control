#include <stdint.h>
#include <stdbool.h>
#include "proj_config.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "debug_link.h"

void config_imu_accel_scale_calib_setting(float gx_extreme_p, float gx_extreme_n,
                float gy_extreme_p, float gy_extreme_n,
                float gz_extreme_p, float gz_extreme_n)
{
	mpu6500_config_scale_calib_setting(gx_extreme_p, gx_extreme_n,
	                                   gy_extreme_p, gy_extreme_n,
	                                   gz_extreme_p, gz_extreme_n);
}

void get_imu_raw_accel(float *accel)
{
	mpu6500_get_raw_accel(accel);
}

void get_imu_filtered_accel(float *accel)
{
	mpu6500_get_filtered_accel(accel);
}

void get_imu_raw_gyro(float *gyro)
{
	mpu6500_get_raw_gyro(gyro);
}

void get_imu_filtered_gyro(float *gyro)
{
	mpu6500_get_filtered_gyro(gyro);
}

void get_imu_compass_raw(float *mag_raw)
{
	ist8310_get_raw_mag(mag_raw);
}

bool is_compass_present(void)
{
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG) || \
    (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
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

float get_imu_compass_raw_strength(void)
{
	return ist8310_get_raw_mag_strength();
}

void send_compass_debug_message(debug_msg_t *payload)
{
	float mag_raw[3] = {0.0f};
	get_imu_compass_raw(mag_raw);

	float mag_strength = get_imu_compass_raw_strength();
	float update_freq = get_imu_compass_update_freq();

	pack_debug_debug_message_header(payload, MESSAGE_ID_COMPASS);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&mag_strength, payload);
	pack_debug_debug_message_float(&update_freq, payload);
}
