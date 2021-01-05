#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "matrix.h"
#include "se3_math.h"

#define EARTH_RADIUS 6371000.0f //[m]

float home_longitude = 0.0f;
float home_latitude = 0.0f;

float home_ecef_x = 0.0f;
float home_ecef_y = 0.0f;
float home_ecef_z = 0.0f;

bool home_is_set = false;

bool gps_home_is_set(void)
{
	return home_is_set;
}

void set_home_longitude_latitude(float longitude, float latitude, float height_msl)
{
	float sin_lambda = arm_sin_f32(deg_to_rad(longitude));
	float cos_lambda = arm_cos_f32(deg_to_rad(longitude));
	float sin_phi = arm_sin_f32(deg_to_rad(latitude));
	float cos_phi = arm_cos_f32(deg_to_rad(latitude));

	home_longitude = longitude;
	home_latitude = latitude;

	home_longitude = longitude;
	home_latitude = latitude;
	home_ecef_x = (height_msl + EARTH_RADIUS) * cos_phi * cos_lambda;
	home_ecef_y = (height_msl + EARTH_RADIUS) * cos_phi * sin_lambda;
	home_ecef_z = (height_msl + EARTH_RADIUS) * sin_phi;

	home_is_set = true;
}

void get_home_longitude_latitude(float *longitude, float *latitude)
{
	*longitude = home_longitude;
	*latitude = home_latitude;
}

void longitude_latitude_to_enu(float longitude, float latitude, float height_msl,
                               float *x_enu, float *y_enu, float *z_enu)
{
	float sin_lambda = sin(deg_to_rad(longitude));
	float cos_lambda = cos(deg_to_rad(longitude));
	float sin_phi = sin(deg_to_rad(latitude));
	float cos_phi = cos(deg_to_rad(latitude));

	/* convert geodatic coordinates to earth center earth fixed frame (ecef) */
	float ecef_now_x = (height_msl + EARTH_RADIUS) * cos_phi * cos_lambda;
	float ecef_now_y = (height_msl + EARTH_RADIUS) * cos_phi * sin_lambda;
	float ecef_now_z = (height_msl + EARTH_RADIUS) * sin_phi;

	/* convert position from earth center earth fixed frame to east north up frame */
	float r11 = -sin_lambda;
	float r12 = cos_lambda;
	float r13 = 0;
	float r21 = -cos_lambda * sin_phi;
	float r22 = -sin_lambda * sin_phi;
	float r23 = cos_phi;
	//float r31 = cos_lambda * cos_phi;
	//float r32 = sin_lambda * cos_phi;
	//float r33 = sin_phi;

	float dx = ecef_now_x - home_ecef_x;
	float dy = ecef_now_y - home_ecef_y;
	float dz = ecef_now_z - home_ecef_z;

	*x_enu = (r11 * dx) + (r12 * dy) + (r13 * dz);
	*y_enu = (r21 * dx) + (r22 * dy) + (r23 * dz);
	*z_enu = height_msl; //barometer of height sensor

	//gps:
	//*z_enu = (r31 * dx) + (r32 * dy) + (r33 * dz);
}
