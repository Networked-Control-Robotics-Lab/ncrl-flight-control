#include "optitrack.h"
#include "vins_mono.h"
#include "position_state.h"
#include "proj_config.h"
#include "ms5611.h"
#include "gps.h"
#include "ins.h"

sensor_manager_t sensor_manager = {
	.heading_src = SELECT_HEADING_SENSOR,
	.height_src = SELECT_HEIGHT_SENSOR,
	.position_src = SELECT_POSITION_SENSOR
};

bool is_xy_position_info_available(void)
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

bool is_height_info_available(void)
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
