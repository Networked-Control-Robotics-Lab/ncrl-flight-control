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
#define EQUATORIAL_RADIUS 6378137 //[m], earth semi-major length (AE)
#define POLAR_RADIUS      6356752 //[m], earth semi-minor length (AP)
#define AP_SQ_DIV_AE_SQ   0.99331 //(AP^2)/(AE^2)
#define ECCENTRICITY      0.08182 //e^2 = 1 - (AP^2)/(AE^2)

/*==========================*
 * spherical model of earth *
 *==========================*/
#define AVERAGE_EARTH_RADIUS 6371000.0f //[m]

int32_t home_longitude_s32 = 0;
int32_t home_latitude_s32 = 0;

double home_longitude = 0.0f;
double home_latitude = 0.0f;

double home_ecef_x = 0.0f;
double home_ecef_y = 0.0f;
double home_ecef_z = 0.0f;

bool home_is_set = false;

bool gps_home_is_set(void)
{
	return home_is_set;
}

void set_home_longitude_latitude(int32_t _longitude, int32_t _latitude, float height_msl)
{
	home_longitude_s32 = _longitude;
	home_latitude_s32 = _latitude;

	double longitude = (double)_longitude * 1e-7;
	double latitude = (double)_latitude * 1e-7;

	home_longitude = longitude;
	home_latitude = latitude;

	double sin_lambda = sinl(deg_to_rad(longitude));
	double cos_lambda = cosl(deg_to_rad(longitude));
	double sin_phi = sinl(deg_to_rad(latitude));
	double cos_phi = cosl(deg_to_rad(latitude));

#if 1   /* ellipsoid */
	double N = EQUATORIAL_RADIUS / sqrtl(1 - (ECCENTRICITY * sin_phi * sin_phi));

	home_ecef_x = (N + height_msl) * cos_phi * cos_lambda;
	home_ecef_y = (N + height_msl) * cos_phi * sin_lambda;
	home_ecef_z = (N * AP_SQ_DIV_AE_SQ + height_msl) * sin_phi;
#else   /* sphere */
	home_ecef_x = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * cos_lambda;
	home_ecef_y = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * sin_lambda;
	home_ecef_z = (height_msl + AVERAGE_EARTH_RADIUS) * sin_phi;
#endif

	home_is_set = true;
}

void get_home_longitude_latitude(int32_t *longitude, int32_t *latitude)
{
	*longitude = home_longitude;
	*latitude = home_latitude;
}

void longitude_latitude_to_enu(int32_t _longitude, int32_t _latitude, float height_msl,
                               float *x_enu, float *y_enu, float *z_enu)
{
	double longitude = (double)_longitude * 1e-7;
	double latitude = (double)_latitude * 1e-7;

	double sin_lambda = sinl(deg_to_rad(longitude));
	double cos_lambda = cosl(deg_to_rad(longitude));
	double sin_phi = sinl(deg_to_rad(latitude));
	double cos_phi = cosl(deg_to_rad(latitude));

	/* convert geodatic coordinates to earth center earth fixed frame (ecef) */
#if 1   /* ellipsoid */
	double N = EQUATORIAL_RADIUS / sqrtl(1 - (ECCENTRICITY * sin_phi * sin_phi));
	double ecef_now_x = (N + height_msl) * cos_phi * cos_lambda;
	double ecef_now_y = (N + height_msl) * cos_phi * sin_lambda;
	double ecef_now_z = (N * AP_SQ_DIV_AE_SQ + height_msl) * sin_phi;
#else   /* sphere */
	double ecef_now_x = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * cos_lambda;
	double ecef_now_y = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * sin_lambda;
	double ecef_now_z = (height_msl + AVERAGE_EARTH_RADIUS) * sin_phi;
#endif

	/* convert position from earth center earth fixed frame to east north up frame */
	double r11 = -sin_lambda;
	double r12 = cos_lambda;
	double r13 = 0;
	double r21 = -cos_lambda * sin_phi;
	double r22 = -sin_lambda * sin_phi;
	double r23 = cos_phi;
	//double r31 = cos_lambda * cos_phi;
	//double r32 = sin_lambda * cos_phi;
	//double r33 = sin_phi;

	double dx = ecef_now_x - home_ecef_x;
	double dy = ecef_now_y - home_ecef_y;
	double dz = ecef_now_z - home_ecef_z;

	*x_enu = (float)((r11 * dx) + (r12 * dy) + (r13 * dz));
	*y_enu = (float)((r21 * dx) + (r22 * dy) + (r23 * dz));
	*z_enu = height_msl; //barometer of height sensor
	//*z_enu = (float)((r31 * dx) + (r32 * dy) + (r33 * dz)); //gps
}
