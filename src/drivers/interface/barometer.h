#ifndef __BAROMETER_H__
#define __BAROMETER_H__

void barometer_update(float *altitude, float *altitude_rate);
void barometer_set_sea_level(void);

#endif
