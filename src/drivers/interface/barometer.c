#include "ms5611.h"

void barometer_update(float *altitude, float *altitude_rate)
{
	ms5611_update(altitude, altitude_rate);
}

void barometer_set_sea_level(void)
{
	ms5611_set_sea_level();
}
