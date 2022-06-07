#include <stdint.h>
#include "ublox_m8n.h"

bool is_gps_available(void)
{
	return ublox_available();
}

void get_gps_longitude_latitude_height_s32(int32_t *longitude, int32_t *latitude, int32_t *height_msl)
{
	ublox_m8n_get_longitude_latitude_height_s32(longitude, latitude, height_msl);
}

void get_gps_velocity_ned(float *vx, float *vy, float *vz)
{
	ublox_m8n_get_velocity_ned(vx, vy, vz);
}

int get_gps_satellite_numbers(void)
{
	return ublox_m8n_get_satellite_numbers();
}

void get_gps_dilution_of_precision(float *pdop, float *hdop, float *vdop)
{
	ublox_m8n_get_dilution_of_precision(pdop, hdop, vdop);
}

uint8_t get_gps_fix_type(void)
{
	return ublox_m8n_get_fix_type();
}

void get_gps_position_uncertainty(float *h_acc, float *v_acc)
{
	ublox_m8n_get_position_uncertainty(h_acc, v_acc);
}

float get_gps_ground_speed(void)
{
	return ublox_m8n_get_ground_speed();
}

float get_gps_heading(void)
{
	return ublox_m8n_get_heading();
}

float get_gps_update_freq(void)
{
	return ublox_m8n_get_update_freq();
}

float get_gps_last_update_time_ms(void)
{
	return ublox_m8n_get_last_update_time_ms();
}

