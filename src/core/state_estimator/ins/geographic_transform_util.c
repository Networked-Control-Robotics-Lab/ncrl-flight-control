#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "matrix.h"
#include "se3_math.h"
#include "geographic_transform_util.h"

/*================================================================*
 * ellipsoid model of earth.                                      *
 * the parameters is given by NASA's Earth Fact Sheet:            *
 * https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html *
 *================================================================*/
#define EQUATORIAL_RADIUS 6378137       //[m], earth semi-major length (AE)
#define POLAR_RADIUS      6356752       //[m], earth semi-minor length (AP)
#define AP_SQ_DIV_AE_SQ   0.9933055218  //(AP^2) / (AE^2)
#define ECCENTRICITY      0.00669447819 //e^2 = 1 - (AP^2)/(AE^2)

/*==========================*
 * spherical model of earth *
 *==========================*/
#define AVERAGE_EARTH_RADIUS 6371000.0f //[m]

/*====================*
 * select earth model *
 *====================*/
#define EARTH_MODEL_SPHERE    0
#define EARTH_MODEL_ELLIPSOID 1
#define SELECT_EARTH_MODEL EARTH_MODEL_ELLIPSOID

gps_home_t gps_home;

bool gps_home_is_set(void)
{
	return gps_home.home_is_set;
}

void geographic_to_ecef_coordinate_transform(double *pos_ecef, double sin_lambda,
                double cos_lambda, double sin_phi,
                double cos_phi, float height_msl)
{
#if (SELECT_EARTH_MODEL == EARTH_MODEL_ELLIPSOID)
	/* ellipsoid earth model */
	double N = EQUATORIAL_RADIUS / sqrtl(1 - (ECCENTRICITY * sin_phi * sin_phi));
	pos_ecef[0] = (N + height_msl) * cos_phi * cos_lambda;
	pos_ecef[1] = (N + height_msl) * cos_phi * sin_lambda;
	pos_ecef[2] = (N * AP_SQ_DIV_AE_SQ + height_msl) * sin_phi;
#else
	/* sphere earth model */
	pos_ecef[0] = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * cos_lambda;
	pos_ecef[1] = (height_msl + AVERAGE_EARTH_RADIUS) * cos_phi * sin_lambda;
	pos_ecef[2] = (height_msl + AVERAGE_EARTH_RADIUS) * sin_phi;
#endif
}

void ecef_to_enu_coordinate_transform(float *pos_enu, double *pos_ecef, double sin_lambda,
                                      double cos_lambda, double sin_phi, double cos_phi,
                                      float height_msl)

{
	double r11 = -sin_lambda;
	double r12 = cos_lambda;
	double r13 = 0;
	double r21 = -cos_lambda * sin_phi;
	double r22 = -sin_lambda * sin_phi;
	double r23 = cos_phi;
	//double r31 = cos_lambda * cos_phi;
	//double r32 = sin_lambda * cos_phi;
	//double r33 = sin_phi;

	double dx = pos_ecef[0] - gps_home.home_ecef[0];
	double dy = pos_ecef[1] - gps_home.home_ecef[1];
	double dz = pos_ecef[2] - gps_home.home_ecef[2];

	pos_enu[0] = (float)((r11 * dx) + (r12 * dy) + (r13 * dz));
	pos_enu[1] = (float)((r21 * dx) + (r22 * dy) + (r23 * dz));
	//pos_enu[2] = (float)((r31 * dx) + (r32 * dy) + (r33 * dz));
	pos_enu[2] = (float)height_msl;
}

void set_home_longitude_latitude(int32_t _longitude, int32_t _latitude, float height_msl)
{
	gps_home.home_longitude_s32 = _longitude;
	gps_home.home_latitude_s32 = _latitude;

	gps_home.home_longitude = (double)_longitude * 1e-7;
	gps_home.home_latitude = (double)_latitude * 1e-7;

	double sin_lambda = sinl(deg_to_rad(gps_home.home_longitude));
	double cos_lambda = cosl(deg_to_rad(gps_home.home_longitude));
	double sin_phi = sinl(deg_to_rad(gps_home.home_latitude));
	double cos_phi = cosl(deg_to_rad(gps_home.home_latitude));

	geographic_to_ecef_coordinate_transform(gps_home.home_ecef, sin_lambda,cos_lambda,
	                                        sin_phi, cos_phi, height_msl);

	gps_home.home_is_set = true;
}

void se_gps_home_longitude_latitude(int32_t *longitude, int32_t *latitude)
{
	*longitude = gps_home.home_longitude;
	*latitude = gps_home.home_latitude;
}

void longitude_latitude_to_enu(float *pos_enu, int32_t _longitude,
                               int32_t _latitude, float height_msl)
{
	double longitude = (double)_longitude * 1e-7;
	double latitude = (double)_latitude * 1e-7;

	double sin_lambda = sinl(deg_to_rad(longitude));
	double cos_lambda = cosl(deg_to_rad(longitude));
	double sin_phi = sinl(deg_to_rad(latitude));
	double cos_phi = cosl(deg_to_rad(latitude));

	double pos_ecef[3];
	geographic_to_ecef_coordinate_transform(pos_ecef, sin_lambda, cos_lambda,
	                                        sin_phi, cos_phi, height_msl);

	ecef_to_enu_coordinate_transform(pos_enu, pos_ecef, sin_lambda, cos_lambda,
	                                 sin_phi, cos_phi, height_msl);
}
