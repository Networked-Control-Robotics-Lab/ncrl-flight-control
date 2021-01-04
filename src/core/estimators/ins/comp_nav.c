#include "debug_link.h"
#include "optitrack.h"
#include "imu.h"
#include "barometer.h"
#include "gps_to_enu.h"
#include "position_sensor.h"
#include "comp_nav.h"
#include "ahrs_selector.h"

/* position (enu) */
float pos_last[3] = {0.0f};
float pos_predict[3] = {0.0f};
float pos_fused[3] = {0.0f};

/* velocity (enu) */
float vel_last[3] = {0.0f};
float vel_predict[3] = {0.0f};
float vel_fused[3] = {0.0f};

float pos_a[3] = {0.5f,    //gps_x
                  0.5f,    //gps_y
                  0.997f
                 }; //barometer or height sensor
float vel_a[3] = {0.2f,    //gps_x
                  0.2f,    //gps_y
                  0.997f
                 }; //barometer or height sensor

float dt = 0;
float half_dt_squared = 0;

void comp_nav_init(float _dt)
{
	dt = _dt;
	half_dt_squared = _dt / 2;
}

void comp_nav_estimate(void)
{
	float pos_enu_raw[3], vel_enu_raw[3];

	/* read barometer height and velocity */
	float barometer_height, barometer_velocity;
	barometer_height = barometer_get_relative_altitude();
	barometer_velocity = barometer_get_relative_altitude_rate();

	/* read gps position and convert it in to enu coordinate frame */
	float gps_longitude, gps_latitude, gps_msl_height;
	get_gps_longitude_latitude_height(&gps_longitude, &gps_latitude, &gps_msl_height);
	longitude_latitude_to_enu(gps_longitude, gps_latitude, gps_msl_height,
	                          &pos_enu_raw[0], &pos_enu_raw[1], &pos_enu_raw[2]);
	pos_enu_raw[2] = barometer_height; //use barometer height

	/* read gps velocity and convert it from ned frame into enu frame */
	float gps_ned_vx, gps_ned_vy, gps_ned_vz;
	get_gps_velocity_ned(&gps_ned_vx, &gps_ned_vy, &gps_ned_vz);
	vel_enu_raw[0] = gps_ned_vy;
	vel_enu_raw[1] = gps_ned_vx;
	pos_enu_raw[2] = barometer_velocity; //use barometer height velocity

	/* read rotation matrix of current attitude */
	float Rt[3*3];
	get_attitude_direction_cosine_matrix((float **)&Rt);

	/* use gps and barometer as complementary filter input */
	pos_vel_complementary_filter(Rt, pos_enu_raw, vel_enu_raw);
}

/* estimate position and velocity using complementary filter */
void pos_vel_complementary_filter(float *Rt, float *pos_enu_raw, float *vel_enu_raw)
{
	/* read accelerometer (which is represented in body-fixed frame) */
	float accel_b_ned[3];
	get_accel_lpf(accel_b_ned);

	/* convert from body-fixed frame to inertial frame */
	float accel_i_ned[3];
	accel_i_ned[0] = Rt[0*3 + 0] * accel_b_ned[0] +
	                 Rt[0*3 + 1] * accel_b_ned[1] +
	                 Rt[0*3 + 2] * accel_b_ned[2];
	accel_i_ned[1] = Rt[1*3 + 0] * accel_b_ned[0] +
	                 Rt[1*3 + 1] * accel_b_ned[1] +
	                 Rt[1*3 + 2] * accel_b_ned[2];
	accel_i_ned[2] = Rt[2*3 + 0] * accel_b_ned[0] +
	                 Rt[2*3 + 1] * accel_b_ned[1] +
	                 Rt[2*3 + 2] * accel_b_ned[2];

	/* gravity compensation */
	accel_i_ned[2] += 9.8f;

	/* covert from ned frame to enu frame */
	float accel_i_enu[3];
	accel_i_enu[0] =  accel_i_ned[1];
	accel_i_enu[1] =  accel_i_ned[0];
	accel_i_enu[2] = -accel_i_ned[2];

	/* kinematic update with accelerometer */
	pos_predict[0] = pos_last[0] + (vel_last[0] * dt) + (accel_i_enu[0] * half_dt_squared);
	pos_predict[1] = pos_last[1] + (vel_last[1] * dt) + (accel_i_enu[1] * half_dt_squared);
	pos_predict[2] = pos_last[2] + (vel_last[2] * dt) + (accel_i_enu[2] * half_dt_squared);
	vel_predict[0] = vel_last[0] + (accel_i_enu[0] * dt);
	vel_predict[1] = vel_last[1] + (accel_i_enu[1] * dt);
	vel_predict[2] = vel_last[2] + (accel_i_enu[2] * dt);

	/* fusion */
	pos_fused[0] = (pos_a[0] * pos_enu_raw[0]) + ((1.0f - pos_a[0]) * pos_predict[0]);
	pos_fused[1] = (pos_a[1] * pos_enu_raw[1]) + ((1.0f - pos_a[1]) * pos_predict[1]);
	pos_fused[2] = (pos_a[2] * pos_enu_raw[2]) + ((1.0f - pos_a[2]) * pos_predict[2]);
	vel_fused[0] = (vel_a[0] * vel_enu_raw[0]) + ((1.0f - vel_a[0]) * vel_predict[0]);
	vel_fused[1] = (vel_a[1] * vel_enu_raw[1]) + ((1.0f - vel_a[1]) * vel_predict[1]);
	vel_fused[2] = (vel_a[2] * vel_enu_raw[2]) + ((1.0f - vel_a[2]) * vel_predict[2]);

	/* save fused result for next iteration */
	pos_last[0] = pos_fused[0];
	pos_last[1] = pos_fused[1];
	pos_last[2] = pos_fused[2];
	vel_last[0] = vel_fused[0];
	vel_last[1] = vel_fused[1];
	vel_last[2] = vel_fused[2];
}

void get_complementary_fused_position(float *pos_enu)
{
	pos_enu[0] = pos_fused[0];
	pos_enu[1] = pos_fused[1];
	pos_enu[2] = pos_fused[2];
}

void get_complementary_fused_velocity(float *vel_enu)
{
	vel_enu[0] = vel_fused[0];
	vel_enu[1] = vel_fused[1];
	vel_enu[2] = vel_fused[2];
}
