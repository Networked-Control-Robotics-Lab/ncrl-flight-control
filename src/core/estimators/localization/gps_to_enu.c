#include "arm_math.h"
#include "matrix.h"

#define EARTH_RADIUS 6371.0f

float home_longitude = 0.0f;
float home_latitude = 0.0f;

float home_ecef_x = 0.0f;
float home_ecef_y = 0.0f;

void set_home_longitude_latitude(float longitude, float latitude)
{
	float sin_lambda = arm_sin_f32(longitude);
	float cos_lambda = arm_cos_f32(longitude);
	float cos_phi = arm_cos_f32(latitude);

	home_longitude = longitude;
	home_latitude = latitude;

	home_ecef_x = EARTH_RADIUS * cos_phi * cos_lambda;
	home_ecef_y = EARTH_RADIUS * cos_phi * sin_lambda;
	//home_ecef_z = EARTH_RADIUS * sin_phi
}

void get_home_longitude_latitude(float *longitude, float *latitude)
{
	*longitude = home_longitude;
	*latitude = home_latitude;
}

void longitude_latitude_to_enu(float longitude, float latitude,
                               float *x_enu, float *y_enu)
{
	float sin_lambda = arm_sin_f32(longitude);
	float cos_lambda = arm_cos_f32(longitude);
	float sin_phi = arm_sin_f32(latitude);
	float cos_phi = arm_cos_f32(latitude);

	/* convert geodatic coordinates to earth center earth fixed frame (ecef) */
	float ecef_now_x = EARTH_RADIUS * cos_phi * cos_lambda;
	float ecef_now_y = EARTH_RADIUS * cos_phi * sin_lambda;
	//float ecef_now_z = EARTH_RADIUS * sin_phi;

	/* convert position from earth center earth fixed frame to east north up frame */
	float r11 = -sin_lambda;
	float r12 = -cos_lambda * sin_phi;
	//float r13 = cos_lambda * cos_phi;
	float r21 = cos_lambda;
	float r22 = -sin_lambda * sin_phi;
	//float r23 = sin_lambda * cos_phi;
	//float r31 = 0.0f;
	//float r32 = cos_phi;
	//float r33 = sin_phi;

	float dx = ecef_now_x - home_ecef_x;
	float dy = ecef_now_y - home_ecef_y;
	//float dz = ecef_now_z - home_ecef_z;

	*x_enu = (r11 * dx) + (r12 * dy);
	*y_enu = (r21 * dx) + (r22 * dy);
}
