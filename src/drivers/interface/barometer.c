#include "proj_config.h"
#include "ms5611.h"

bool is_barometer_available(void)
{
	return ms5611_available();
}

void barometer_wait_until_stable(void)
{
#if (ENABLE_BAROMETER != 0)
	ms5611_wait_until_stable();
#endif
}

void barometer_set_sea_level(void)
{
	ms5611_set_sea_level();
}

float barometer_get_pressure(void)
{
	return ms5611_get_pressure();
}

float barometer_get_relative_altitude(void)
{
	return ms5611_get_relative_altitude();
}

float barometer_get_relative_altitude_rate(void)
{
	return ms5611_get_relative_altitude_rate();
}

float barometer_get_update_freq(void)
{
	return ms5611_get_update_freq();
}
