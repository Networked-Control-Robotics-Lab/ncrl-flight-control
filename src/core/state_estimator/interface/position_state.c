#include "optitrack.h"
#include "position_state.h"
#include "proj_config.h"
#include "ms5611.h"
#include "gps.h"
#include "ins.h"

bool is_xy_position_info_available(void)
{
#if (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_OPTITRACK)
	return optitrack_available();
#elif (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_GPS)
	return is_gps_available();
#else
	return false;
#endif
}

bool is_height_info_available(void)
{
#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_OPTITRACK)
	return optitrack_available();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	return is_barometer_available();
#else
	return false;
#endif
}

void get_enu_position(float *pos)
{
	/* x-y position */
#if (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_OPTITRACK)
	pos[0] = optitrack_read_pos_x();
	pos[1] = optitrack_read_pos_y();
#elif (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_GPS)
	pos[0] = ins_get_fused_position_x();
	pos[1] = ins_get_fused_position_y();
#else
	pos[0] = 0.0f;
	pos[1] = 0.0f;
#endif

	/* z position */
#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_OPTITRACK)
	pos[2] = optitrack_read_pos_z();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	pos[2] = ins_get_fused_position_z();
#else
	pos[2] = 0.0f;
#endif
}

float get_enu_height(void)
{
#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_OPTITRACK)
	return optitrack_read_pos_z();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	return ins_get_fused_position_z();
#else
	return 0.0f;
#endif
}

void get_enu_velocity(float *vel)
{
	/* x-y velocity */
#if (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_OPTITRACK)
	vel[0] = optitrack_read_vel_x();
	vel[1] = optitrack_read_vel_y();
#elif (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_GPS)
	vel[0] = ins_get_fused_velocity_x();
	vel[1] = ins_get_fused_velocity_y();
#else
	vel[0] = 0.0f;
	vel[1] = 0.0f;
#endif

	/* z velocity */
#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_OPTITRACK)
	vel[2] = optitrack_read_vel_z();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	vel[2] = ins_get_fused_velocity_z();
#else
	vel[2] = 0.0f;
#endif
}
