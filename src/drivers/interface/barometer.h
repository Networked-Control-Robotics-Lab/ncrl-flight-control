#ifndef __BAROMETER_H__
#define __BAROMETER_H__

bool is_barometer_available(void);
void barometer_wait_until_stable(void);

void barometer_set_sea_level(void);

float barometer_get_pressure(void);
float barometer_get_relative_altitude(void);
float barometer_get_relative_altitude_rate(void);
float barometer_get_update_freq(void);

#endif
