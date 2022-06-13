#include <stdbool.h>
#include <stdint.h>
#include "geographic_transform.h"
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
#include "vio.h"
#include "ahrs_eskf.h"

#define INS_LOOP_PERIOD 0.0025f //400Hz

/* estimation of the attitude */
attitude_t attitude;

/* raw position and velocity */
float pos_raw_ned[3];
float vel_raw_ned[3];

/* fused position and velocity */
float pos_fused_ned[3];
float vel_fused_ned[3];

/* ins_eskf convergence status */
bool eskf_status_last = false;

void ins_init(void)
{
	ins_comp_filter_init(INS_LOOP_PERIOD);
	ins_eskf_init(INS_LOOP_PERIOD);
}

void ins_state_decoupled_estimation(void)
{
	/* decoupled orientation state estimation */
	ahrs_estimate(&attitude);

	/* decoupled position state estimation with complementary filter */
	ins_complementary_filter_estimate(pos_raw_ned, vel_raw_ned, pos_fused_ned, vel_fused_ned);
}

void ins_full_state_estimation(void)
{
	/* full state estimation with eskf */
	ins_eskf_estimate(&attitude, pos_raw_ned, vel_raw_ned, pos_fused_ned, vel_fused_ned);

	/* check convergence of the ins_eskf */
	bool eskf_status = ins_eskf_is_stable();

	/* reset process covariance matrix if ins_eskf is suspended */
	if(eskf_status_last == true && eskf_status == false) {
		ins_eskf_reset_process_covariance_matrix();
	}
	eskf_status_last = eskf_status;

	/* if sensor signal lost causes the full state estimation impossible, we
	 * estimate the state independently */
	if(eskf_status == true) {
		/* synchronize ins_eskf quaternion state with ahrs_eskf */
		set_ahrs_eskf_quaternion(attitude.q);

		/* synchronize ins_eskf position state with complementary filter */
		set_ins_complementary_filter_state(pos_fused_ned, vel_fused_ned);
	} else {
		/* ins is not ready, execute ahrs algorithm to estimate the attitude */
		ahrs_estimate(&attitude); //TODO: restrict ahrs algorithm to eskf only

		/* estimate height with complementary filter if height info is available */
		ins_complementary_filter_estimate(pos_raw_ned, vel_raw_ned,
		                                  pos_fused_ned, vel_fused_ned);
	}
}

static void ins_led_state_update(void)
{
	switch(get_position_sensor()) {
	case POSITION_FUSION_USE_OPTITRACK:
		set_rgb_led_service_navigation_on_flag(optitrack_available());
		break;
	case POSITION_FUSION_USE_VINS_MONO:
		set_rgb_led_service_navigation_on_flag(vio_available());
		break;
	case POSITION_FUSION_USE_GPS:
#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
		set_rgb_led_service_navigation_on_flag(ins_complementary_filter_ready());
#elif (SELECT_INS == INS_ESKF)
		set_rgb_led_service_navigation_on_flag(ins_eskf_is_stable());
#endif
		break;
	default:
		set_rgb_led_service_navigation_on_flag(false);
	}
}

void ins_state_estimate(void)
{
#if (SELECT_INS == INS_COMPLEMENTARY_FILTER)
	ins_state_decoupled_estimation();
#elif (SELECT_INS == INS_ESKF)
	ins_full_state_estimation();
#endif

	ins_led_state_update();
}

void ins_get_raw_position_enu(float *pos)
{
	pos[0] =  pos_raw_ned[1];
	pos[1] =  pos_raw_ned[0];
	pos[2] = -pos_raw_ned[2];
}

float ins_get_raw_position_enu_x(void)
{
	return pos_raw_ned[1];
}

float ins_get_raw_position_enu_y(void)
{
	return pos_raw_ned[0];
}

float ins_get_raw_position_enu_z(void)
{
	return -pos_raw_ned[2];
}

void ins_get_raw_velocity_enu(float *vel)
{
	vel[0] =  vel_raw_ned[1];
	vel[1] =  vel_raw_ned[0];
	vel[2] = -vel_raw_ned[2];
}

float ins_get_raw_velocity_enu_x(void)
{
	return vel_raw_ned[1];
}

float ins_get_raw_velocity_enu_y(void)
{
	return vel_raw_ned[0];
}

float ins_get_raw_velocity_enu_z(void)
{
	return -vel_raw_ned[2];
}

void ins_get_fused_position_enu(float *pos)
{
	pos[0] =  pos_fused_ned[1];
	pos[1] =  pos_fused_ned[0];
	pos[2] = -pos_fused_ned[2];
}

float ins_get_fused_position_enu_x(void)
{
	return pos_fused_ned[1];
}

float ins_get_fused_position_enu_y(void)
{
	return pos_fused_ned[0];
}

float ins_get_fused_position_enu_z(void)
{
	return -pos_fused_ned[2];
}

void ins_get_fused_velocity_enu(float *vel)
{
	vel[0] =  vel_fused_ned[1];
	vel[1] =  vel_fused_ned[0];
	vel[2] = -vel_fused_ned[2];
}

float ins_get_fused_velocity_enu_x(void)
{
	return vel_fused_ned[1];
}

float ins_get_fused_velocity_enu_y(void)
{
	return vel_fused_ned[0];
}

float ins_get_fused_velocity_enu_z(void)
{
	return -vel_fused_ned[2];
}

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
