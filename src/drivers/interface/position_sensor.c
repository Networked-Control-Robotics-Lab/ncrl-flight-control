#include "optitrack.h"
#include "position_sensor.h"

bool is_xy_position_info_available(void)
{
	return optitrack_available();
}

bool is_height_info_available(void)
{
	return optitrack_available();
}

void get_enu_position(float *pos)
{
	optitrack_read_pos_x(&pos[0]);
	optitrack_read_pos_y(&pos[1]);
	optitrack_read_pos_z(&pos[2]);
}

float get_enu_height(void)
{
	float height;
	optitrack_read_pos_z(&height);
	return height;
}

void get_wgs84_position(float *latitude, float *longtitude, float *height)
{
}

void get_enu_velocity(float *vel)
{
	optitrack_read_vel_x(&vel[0]);
	optitrack_read_vel_y(&vel[1]);
	optitrack_read_vel_z(&vel[2]);
}
