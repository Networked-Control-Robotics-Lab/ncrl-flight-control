#ifndef __LOCALIZATION_SYSTEM_H__
#define __LOCALIZATION_SYSTEM_H__

bool is_localization_info_available(void);

void get_enu_position(float *pos);
void get_wgs84_position(float *latitude, float *longtitude, float *height);
void get_enu_velocity(float *vel);

#endif
