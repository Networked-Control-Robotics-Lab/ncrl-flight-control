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

float pos_raw[3];
float vel_raw[3];
float pos_fused[3];
float vel_fused[3];

void ins_init(void)
{
	ins_comp_filter_init(INS_LOOP_PERIOD);
	eskf_ins_init(INS_LOOP_PERIOD);
}

static void ins_decoupled_state_estimation(void)
{
	/* decoupled orientation state estimation */
	ahrs_estimate(&attitude);

	/* decoupled position state estimation with complementary filter */
	ins_complementary_filter_estimate(pos_raw, vel_raw, pos_fused, vel_fused);
}

static void ins_full_state_estimation(void)
{
	/* full state estimation with eskf */
	ins_eskf_estimate(&attitude, pos_raw, vel_raw, pos_fused, vel_fused);

	/* call decoupled state estimation if eskf is not ready */
	if(eskf_ins_is_stable() == false) {
		ins_decoupled_state_estimation();
	}
}

void ins_state_estimate(void)
{
#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
	ins_decoupled_state_estimation();
#elif (SELECT_INS == INS_ESKF)
	ins_full_state_estimation();
#endif
}

/* raw position getters */
void ins_get_raw_position(float *pos)
{
	pos[0] = pos_raw[0];
	pos[1] = pos_raw[1];
	pos[2] = pos_raw[2];
}

float ins_get_raw_position_x(void)
{
	return pos_raw[0];
}

float ins_get_raw_position_y(void)
{
	return pos_raw[1];
}

float ins_get_raw_position_z(void)
{
	return pos_raw[2];
}

/* raw velocity getters */
void ins_get_raw_velocity(float *vel)
{
	vel[0] = vel_raw[0];
	vel[1] = vel_raw[1];
	vel[2] = vel_raw[2];
}

float ins_get_raw_velocity_x(void)
{
	return vel_raw[0];
}

float ins_get_raw_velocity_y(void)
{
	return vel_raw[1];
}

float ins_get_raw_velocity_z(void)
{
	return vel_raw[2];
}

/* ins fused position getters */
void ins_get_fused_position(float *pos)
{
	pos[0] = pos_fused[0];
	pos[1] = pos_fused[1];
	pos[2] = pos_fused[2];
}

float ins_get_fused_position_x(void)
{
	return pos_fused[0];
}

float ins_get_fused_position_y(void)
{
	return pos_fused[1];
}

float ins_get_fused_position_z(void)
{
	return pos_fused[2];
}

/* ins fused velocity getters */
void ins_get_fused_velocity(float *vel)
{
	vel[0] = vel_fused[0];
	vel[1] = vel_fused[1];
	vel[2] = vel_fused[2];
}

float ins_get_fused_velocity_x(void)
{
	return vel_fused[0];
}

float ins_get_fused_velocity_y(void)
{
	return vel_fused[1];
}

float ins_get_fused_velocity_z(void)
{
	return vel_fused[2];
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
