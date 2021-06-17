#include "optitrack.h"
#include "vins_mono.h"
#include "system_state.h"
#include "proj_config.h"
#include "ms5611.h"
#include "gps.h"
#include "ins.h"
#include "autopilot.h"
#include "ist8310.h"
#include "barometer.h"

sensor_manager_t sensor_manager = {
	.heading_src = SELECT_HEADING_SENSOR,
	.height_src = SELECT_HEIGHT_SENSOR,
	.position_src = SELECT_POSITION_SENSOR
};

bool is_heading_available(void)
{
	switch(sensor_manager.heading_src) {
	case HEADING_FUSION_USE_COMPASS:
		return ist8310_available();
	case HEADING_FUSION_USE_OPTITRACK:
		return optitrack_available();
	case HEADING_FUSION_USE_VINS_MONO:
		return vins_mono_available();
	default:
		return false;
	}
}

bool is_xy_position_available(void)
{
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		return optitrack_available();
	case POSITION_FUSION_USE_GPS:
		return is_gps_available();
	case POSITION_FUSION_USE_VINS_MONO:
		return vins_mono_available();
	default:
		return false;
	}
}

bool is_height_available(void)
{
	switch(sensor_manager.height_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		return optitrack_available();
	case HEIGHT_FUSION_USE_BAROMETER:
		return is_barometer_available();
	case HEIGHT_FUSION_USE_VINS_MONO:
		return vins_mono_available();
	default:
		return false;
	}
}

void get_attitude_euler_angles(float *roll, float *pitch, float *yaw)
{
	ins_ahrs_get_attitude_euler_angles(roll, pitch, yaw);
}

void get_attitude_quaternion(float *q)
{
	ins_ahrs_get_attitude_quaternion(q);
}

void get_rotation_matrix_b2i(float **R_b2i)
{
	ins_ahrs_get_rotation_matrix_b2i(R_b2i);
}

void get_rotation_matrix_i2b(float **R_i2b)
{
	ins_ahrs_get_rotation_matrix_i2b(R_i2b);
}

void get_enu_position(float *pos)
{
	/* x-y position */
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		pos[0] = optitrack_read_pos_x();
		pos[1] = optitrack_read_pos_y();
		break;
	case POSITION_FUSION_USE_VINS_MONO:
		pos[0] = vins_mono_read_pos_x();
		pos[1] = vins_mono_read_pos_y();
		break;
	case POSITION_FUSION_USE_GPS:
		pos[0] = ins_get_fused_position_x();
		pos[1] = ins_get_fused_position_y();
		break;
	default:
		pos[0] = 0.0f;
		pos[1] = 0.0f;
		break;
	}

	/* z position */
	switch(sensor_manager.height_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		pos[2] = optitrack_read_pos_z();
		break;
	case HEIGHT_FUSION_USE_VINS_MONO:
		pos[2] = vins_mono_read_pos_z();
		break;
	case HEIGHT_FUSION_USE_BAROMETER:
		pos[2] = ins_get_fused_position_z();
		break;
	default:
		pos[2] = 0.0f;
		break;
	}
}

float get_enu_position_x(void)
{
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		return optitrack_read_pos_x();
	case POSITION_FUSION_USE_VINS_MONO:
		return vins_mono_read_pos_x();
	case POSITION_FUSION_USE_GPS:
		return ins_get_fused_position_x();
	default:
		return 0.0f;
	}
}

float get_enu_position_y(void)
{
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		return optitrack_read_pos_y();
	case POSITION_FUSION_USE_VINS_MONO:
		return vins_mono_read_pos_y();
	case POSITION_FUSION_USE_GPS:
		return ins_get_fused_position_y();
	default:
		return 0.0f;
	}
}

float get_enu_position_z(void)
{
	switch(sensor_manager.height_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		return optitrack_read_pos_z();
	case HEIGHT_FUSION_USE_VINS_MONO:
		return vins_mono_read_pos_z();
	case HEIGHT_FUSION_USE_BAROMETER:
		return ins_get_fused_position_z();
	default:
		return 0.0f;
	}
}

void get_enu_velocity(float *vel)
{
	/* x-y velocity */
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		vel[0] = optitrack_read_vel_x();
		vel[1] = optitrack_read_vel_y();
		break;
	case POSITION_FUSION_USE_VINS_MONO:
		vel[0] = vins_mono_read_vel_x();
		vel[1] = vins_mono_read_vel_y();
		break;
	case POSITION_FUSION_USE_GPS:
		vel[0] = ins_get_fused_velocity_x();
		vel[1] = ins_get_fused_velocity_y();
		break;
	default:
		vel[0] = 0.0f;
		vel[1] = 0.0f;
		break;
	}

	/* z velocity */
	switch(sensor_manager.height_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		vel[2] = optitrack_read_vel_z();
		break;
	case HEIGHT_FUSION_USE_VINS_MONO:
		vel[2] = vins_mono_read_vel_z();
		break;
	case HEIGHT_FUSION_USE_BAROMETER:
		vel[2] = ins_get_fused_velocity_z();
		break;
	default:
		vel[2] = 0.0f;
		break;
	}
}

float get_enu_velocity_x(void)
{
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		return optitrack_read_vel_x();
	case POSITION_FUSION_USE_VINS_MONO:
		return vins_mono_read_vel_x();
	case POSITION_FUSION_USE_GPS:
		return ins_get_fused_velocity_x();
	default:
		return 0.0f;
	}
}

float get_enu_velocity_y(void)
{
	switch(sensor_manager.position_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		return optitrack_read_vel_y();
	case POSITION_FUSION_USE_VINS_MONO:
		return vins_mono_read_vel_y();
	case POSITION_FUSION_USE_GPS:
		return ins_get_fused_velocity_y();
	default:
		return 0.0f;
	}
}

float get_enu_velocity_z(void)
{
	switch(sensor_manager.height_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		return optitrack_read_vel_z();
	case HEIGHT_FUSION_USE_VINS_MONO:
		return vins_mono_read_vel_z();
	case HEIGHT_FUSION_USE_BAROMETER:
		return ins_get_fused_velocity_z();
	default:
		return 0.0f;
	}
}

void change_heading_sensor_src(int new_src)
{
	if(sensor_manager.heading_src == new_src) {
		return;
	}

	/* check if the sensor to switch is currently available */
	bool sensor_available = false;

	switch(new_src) {
	case HEADING_FUSION_USE_COMPASS:
		sensor_available = ist8310_available();
		break;
	case HEADING_FUSION_USE_OPTITRACK:
		sensor_available = optitrack_available();
		break;
	case HEADING_FUSION_USE_VINS_MONO:
		sensor_available = vins_mono_available();
		break;
	default:
		sensor_available = false;
		break;
	}

	if(sensor_available == false) {
		return;
	}

	sensor_manager.heading_src = new_src;
}

void change_height_sensor_src(int new_src)
{
	if(sensor_manager.height_src == new_src) {
		return;
	}

	/* check if the sensor to switch is currently available */
	bool sensor_available = false;

	switch(new_src) {
	case HEIGHT_FUSION_USE_OPTITRACK:
		sensor_available = optitrack_available();
		break;
	case HEIGHT_FUSION_USE_BAROMETER:
		sensor_available = is_barometer_available();
		break;
	case HEIGHT_FUSION_USE_VINS_MONO:
		sensor_available = vins_mono_available();
		break;
	default:
		sensor_available = false;
		break;
	}

	if(sensor_available == false) {
		return;
	}

	/* calculate position error */
	float curr_pos_z;
	curr_pos_z = get_enu_position_z();

	float des_pos_z;
	des_pos_z = autopilot_get_pos_setpoint_z();

	float pos_error_z;
	pos_error_z = curr_pos_z - des_pos_z;

	/* calculate velocity error */
	float curr_vel_z;
	curr_vel_z = get_enu_velocity_z();

	float des_vel_z;
	des_vel_z = autopilot_get_vel_setpoint_z();

	float vel_error_z;
	vel_error_z = curr_vel_z - des_vel_z;

	/* switch to new sensor source */
	sensor_manager.height_src = new_src;

	curr_pos_z = get_enu_position_z();

	//XXX: ignored the rotation
	/* coordinate transform of the desire value */
	autopilot_assign_pos_target_z(curr_pos_z - pos_error_z);
	autopilot_assign_vel_target_z(curr_vel_z - vel_error_z);
}

void change_position_sensor_src(int new_src)
{
	if(sensor_manager.position_src == new_src) {
		return;
	}

	/* check if the sensor to switch is currently available */
	bool sensor_available = false;

	switch(new_src) {
	case POSITION_FUSION_USE_OPTITRACK:
		sensor_available = optitrack_available();
		break;
	case POSITION_FUSION_USE_GPS:
		sensor_available = is_gps_available();
		break;
	case POSITION_FUSION_USE_VINS_MONO:
		sensor_available = vins_mono_available();
		break;
	default:
		sensor_available = false;
		break;
	}

	if(sensor_available == false) {
		return;
	}

	/* calculate position error */
	float curr_pos_x, curr_pos_y;
	curr_pos_x = get_enu_position_x();
	curr_pos_y = get_enu_position_y();

	float des_pos_x, des_pos_y;
	des_pos_x = autopilot_get_pos_setpoint_x();
	des_pos_y = autopilot_get_pos_setpoint_y();

	float pos_error_x, pos_error_y;
	pos_error_x = curr_pos_x - des_pos_x;
	pos_error_y = curr_pos_y - des_pos_y;

	/* calculate velocity error */
	float curr_vel_x, curr_vel_y;
	curr_vel_x = get_enu_velocity_x();
	curr_vel_y = get_enu_velocity_y();

	float des_vel_x, des_vel_y;
	des_vel_x = autopilot_get_vel_setpoint_x();
	des_vel_y = autopilot_get_vel_setpoint_y();

	float vel_error_x, vel_error_y;
	vel_error_x = curr_vel_x - des_vel_x;
	vel_error_y = curr_vel_y - des_vel_y;

	/* switch to new sensor source */
	sensor_manager.position_src = new_src;

	des_pos_x = autopilot_get_pos_setpoint_x();
	des_pos_y = autopilot_get_pos_setpoint_y();

	//XXX: ignored the rotation
	/* coordinate transform of the desire value */
	autopilot_assign_pos_target_x(curr_pos_x - pos_error_x);
	autopilot_assign_pos_target_y(curr_pos_y - pos_error_y);
	autopilot_assign_vel_target_x(curr_vel_x - vel_error_x);
	autopilot_assign_vel_target_y(curr_vel_y - vel_error_y);
}
