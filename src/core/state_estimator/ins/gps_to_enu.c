#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "matrix.h"
#include "se3_math.h"

/*================================================================*
 * ellipsoid model of earth.                                      *
 * the parameters is given by NASA's Earth Fact Sheet:            *
 * https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html *
 *================================================================*/
#define EQUATORIAL_RADIUS 6378137   //[m], earth semi-major length (AE)
#define POLAR_RADIUS      6356752   //[m], earth semi-minor length (AP)
#define AP_SQ_DIV_AE_SQ   0.99331   //(AP^2)/(AE^2)
#define ECCENTRICITY      0.0066945 //e^2 = 1 - (AP^2)/(AE^2)

/*==========================*
 * spherical model of earth *
 *==========================*/
#define AVERAGE_EARTH_RADIUS 6371000.0f //[m]

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
	home_longitude = longitude;
	home_latitude = latitude;

#if 1
	float sin_lambda = sin(deg_to_rad(longitude));
	float cos_lambda = cos(deg_to_rad(longitude));
	float sin_phi = sin(deg_to_rad(latitude));
	float cos_phi = cos(deg_to_rad(latitude));
#else
	float sin_lambda = arm_sin_f32(deg_to_rad(longitude));
	float cos_lambda = arm_cos_f32(deg_to_rad(longitude));
	float sin_phi = arm_sin_f32(deg_to_rad(latitude));
	float cos_phi = arm_cos_f32(deg_to_rad(latitude));
#endif

#if 1   /* ellipsoid */
	float N = EQUATORIAL_RADIUS / sqrt(1 - (ECCENTRICITY * sin_phi * sin_phi));

	home_ecef_x = (N + height_msl) * cos_phi * cos_lambda;
	home_ecef_y = (N + height_msl) * cos_phi * sin_lambda;
	home_ecef_z = (AP_SQ_DIV_AE_SQ + height_msl) * sin_phi;
#else   /* sphere */
	home_ecef_x = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * cos_lambda;
	home_ecef_y = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * sin_lambda;
	home_ecef_z = (height_msl + AVERAGE_EARTH_RADIUS) * sin_phi;
#endif

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
#if 1   /* ellipsoid */
	float N = EQUATORIAL_RADIUS / sqrt(1 - (ECCENTRICITY * sin_phi * sin_phi));
	float ecef_now_x = (N + height_msl) * cos_phi * cos_lambda;
	float ecef_now_y = (N + height_msl) * cos_phi * sin_lambda;
	float ecef_now_z = (AP_SQ_DIV_AE_SQ + height_msl) * sin_phi;
#else   /* sphere */
	float ecef_now_x = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * cos_lambda;
	float ecef_now_y = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * sin_lambda;
	float ecef_now_z = (height_msl + AVERAGE_EARTH_RADIUS) * sin_phi;
#endif

	/* convert position from earth center earth fixed frame to east north up frame */
	float r11 = -sin_phi * sin_lambda;
	float r12 = -sin_phi * cos_lambda;
	float r13 = -cos_phi;
	float r21 = cos_lambda;
	float r22 = -sin_phi;
	float r23 = 0;
	//float r31 = -cos_phi * sin_lambda;
	//float r32 = -cos_phi * cos_lambda;
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
