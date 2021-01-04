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

/* velocity (enu) */
float vel_last[3] = {0.0f};

float pos_a[3];
float vel_a[3];

float dt = 0;
float half_dt_squared = 0;

void comp_nav_init(float _dt)
{
	dt = _dt;
	half_dt_squared = _dt / 2.0f;

	/* position fusion weights */
	pos_a[0] = 0.5f;   //weight of using gps raw x position
	pos_a[1] = 0.5f;   //weight of using gps raw y position
	pos_a[2] = 0.997f; //weight of using barometer height

	/* velocity fusion weights */
	vel_a[0] = 0.2f;   //weight of using gps raw x velocity
	vel_a[1] = 0.2f;   //weight of using gps raw y velocity
	vel_a[2] = 0.997f; //weight of using barometer height velocity
}

/* estimate position and velocity using complementary filter */
void pos_vel_complementary_filter_predict(float *pos_enu_out, float *vel_enu_out)
{
	/* read rotation matrix of current attitude */
	float *Rt;
	get_attitude_direction_cosine_matrix(&Rt);

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
	pos_enu_out[0] = pos_last[0] + (vel_last[0] * dt) + (accel_i_enu[0] * half_dt_squared);
	pos_enu_out[1] = pos_last[1] + (vel_last[1] * dt) + (accel_i_enu[1] * half_dt_squared);
	pos_enu_out[2] = pos_last[2] + (vel_last[2] * dt) + (accel_i_enu[2] * half_dt_squared);
	vel_enu_out[0] = vel_last[0] + (accel_i_enu[0] * dt);
	vel_enu_out[1] = vel_last[1] + (accel_i_enu[1] * dt);
	vel_enu_out[2] = vel_last[2] + (accel_i_enu[2] * dt);

	/* save fused result for next iteration */
	pos_last[0] = pos_enu_out[0];
	pos_last[1] = pos_enu_out[1];
	pos_last[2] = pos_enu_out[2];
	vel_last[0] = vel_enu_out[0];
	vel_last[1] = vel_enu_out[1];
	vel_last[2] = vel_enu_out[2];
}

/* estimate position and velocity using complementary filter */
void pos_vel_complementary_filter_correct(float *pos_enu_in,  float *vel_enu_in,
                float *pos_enu_out, float *vel_enu_out)
{
	/* fusion */
	pos_enu_out[0] = (pos_a[0] * pos_enu_in[0]) + ((1.0f - pos_a[0]) * pos_last[0]);
	pos_enu_out[1] = (pos_a[1] * pos_enu_in[1]) + ((1.0f - pos_a[1]) * pos_last[1]);
	pos_enu_out[2] = (pos_a[2] * pos_enu_in[2]) + ((1.0f - pos_a[2]) * pos_last[2]);
	vel_enu_out[0] = (vel_a[0] * vel_enu_in[0]) + ((1.0f - vel_a[0]) * vel_last[0]);
	vel_enu_out[1] = (vel_a[1] * vel_enu_in[1]) + ((1.0f - vel_a[1]) * vel_last[1]);
	vel_enu_out[2] = (vel_a[2] * vel_enu_in[2]) + ((1.0f - vel_a[2]) * vel_last[2]);

	/* save fused result for next iteration */
	pos_last[0] = pos_enu_out[0];
	pos_last[1] = pos_enu_out[1];
	pos_last[2] = pos_enu_out[2];
	vel_last[0] = vel_enu_out[0];
	vel_last[1] = vel_enu_out[1];
	vel_last[2] = vel_enu_out[2];
}
