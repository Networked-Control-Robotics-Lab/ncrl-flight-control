#include <stdbool.h>
#include <stdint.h>
#include "gps_to_enu.h"
#include "system_state.h"
#include "barometer.h"
#include "compass.h"
#include "ins_comp_filter.h"
#include "gps.h"
#include "led.h"
#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "proj_config.h"
#include "ublox_m8n.h"
#include "ins_sensor_sync.h"
#include "sys_time.h"
#include "lpf.h"
#include "led.h"
#include "optitrack.h"
#include "ahrs.h"
#include "ins_eskf.h"
#include "autopilot.h"

#define INS_LOOP_PERIOD 0.0025f //400Hz

attitude_t attitude;

float pos_enu_raw[3];
float vel_enu_raw[3];
float pos_enu_fused[3];
float vel_enu_fused[3];

/* gps position and velocity low pass filtering */
float gps_px_lpf_gain;
float gps_py_lpf_gain;
float gps_vx_lpf_gain;
float gps_vy_lpf_gain;

float gps_px_lpf;
float gps_py_lpf;
float gps_vx_lpf;
float gps_vy_lpf;

void ins_init(void)
{
	//sampling time = 0.2s (5Hz), cutoff frequency = 10Hz
	lpf_first_order_init(&gps_px_lpf_gain, 0.2, 1);
	lpf_first_order_init(&gps_py_lpf_gain, 0.2, 1);
	lpf_first_order_init(&gps_vx_lpf_gain, 0.2, 1);
	lpf_first_order_init(&gps_vy_lpf_gain, 0.2, 1);

	ins_comp_filter_init(INS_LOOP_PERIOD);
	eskf_ins_init(INS_LOOP_PERIOD);
}

void ins_state_estimate(void)
{
#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
	ahrs_estimate(&attitude);
	ins_complementary_filter_estimate(pos_enu_raw, vel_enu_raw,
	                                  pos_enu_fused, vel_enu_fused);
#elif (SELECT_INS == INS_ESKF)
	/* full state estimator */
	bool eskf_ready = ins_eskf_estimate(&attitude, pos_enu_raw, vel_enu_raw,
	                                    pos_enu_fused, vel_enu_fused);
	eskf_ready &= eskf_ins_is_stable(); //check if the covariance matrix norm is converged

	/* switch to complementary filter */
	if(eskf_ready == false) {
		ahrs_estimate(&attitude);
		ins_complementary_filter_estimate(pos_enu_raw, vel_enu_raw,
		                                  pos_enu_fused, vel_enu_fused);

		/* set flight mode to manual mode */
		autopilot_set_mode(AUTOPILOT_MANUAL_FLIGHT_MODE);
	}
#endif
}

/* raw position getters */
void ins_get_raw_position(float *pos)
{
	pos[0] = pos_enu_raw[0];
	pos[1] = pos_enu_raw[1];
	pos[2] = pos_enu_raw[2];
}

float ins_get_raw_position_x(void)
{
	return pos_enu_raw[0];
}

float ins_get_raw_position_y(void)
{
	return pos_enu_raw[1];
}

float ins_get_raw_position_z(void)
{
	return pos_enu_raw[2];
}

/* raw velocity getters */
void ins_get_raw_velocity(float *vel)
{
	vel[0] = vel_enu_raw[0];
	vel[1] = vel_enu_raw[1];
	vel[2] = vel_enu_raw[2];
}

float ins_get_raw_velocity_x(void)
{
	return vel_enu_raw[0];
}

float ins_get_raw_velocity_y(void)
{
	return vel_enu_raw[1];
}

float ins_get_raw_velocity_z(void)
{
	return vel_enu_raw[2];
}

/* ins fused position getters */
void ins_get_fused_position(float *pos)
{
	pos[0] = pos_enu_fused[0];
	pos[1] = pos_enu_fused[1];
	pos[2] = pos_enu_fused[2];
}

float ins_get_fused_position_x(void)
{
	return pos_enu_fused[0];
}

float ins_get_fused_position_y(void)
{
	return pos_enu_fused[1];
}

float ins_get_fused_position_z(void)
{
	return pos_enu_fused[2];
}

/* ins fused velocity getters */
void ins_get_fused_velocity(float *vel)
{
	vel[0] = vel_enu_fused[0];
	vel[1] = vel_enu_fused[1];
	vel[2] = vel_enu_fused[2];
}

float ins_get_fused_velocity_x(void)
{
	return vel_enu_fused[0];
}

float ins_get_fused_velocity_y(void)
{
	return vel_enu_fused[1];
}

float ins_get_fused_velocity_z(void)
{
	return vel_enu_fused[2];
}

/* ins ahrs attitude getters */
void ins_ahrs_get_attitude_euler_angles(float *roll, float *pitch, float *yaw)
{
	*roll = attitude.roll;
	*pitch = attitude.pitch;
	*yaw = attitude.yaw;
}

void ins_ahrs_get_attitude_quaternion(float *q)
{
	q[0] = attitude.q[0];
	q[1] = attitude.q[1];
	q[2] = attitude.q[2];
	q[3] = attitude.q[3];
}

void ins_ahrs_get_rotation_matrix_b2i(float **R_b2i)
{
	*R_b2i = attitude.R_b2i;
}

void ins_ahrs_get_rotation_matrix_i2b(float **R_i2b)
{
	*R_i2b = attitude.R_i2b;
}
