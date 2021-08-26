#include "proj_config.h"
#include "lidar_lite.h"

bool rangefinder_available(void)
{
	return lidar_lite_available();
}

float rangefinder_get_distance(void)
{
	return lidar_lite_get_distance();
}

float rangefinder_get_velocity(void)
{
	return lidar_lite_get_velocity();
}

float rangefinder_get_update_freq(void)
{
	return lidar_lite_get_update_freq();
}

