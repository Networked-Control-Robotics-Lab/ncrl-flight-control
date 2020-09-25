#ifndef __POSITION_SENSOR_H__
#define __POSITION_SENSOR_H__

bool is_xy_position_info_available(void);
bool is_height_info_available(void);

void get_enu_position(float *pos);
float get_enu_height(void);
void get_wgs84_position(float *latitude, float *longtitude, float *height);
void get_enu_velocity(float *vel);

void get_gps_longitude_latitude_height(float *longitude, float *latitude, float *height);
void get_gps_velocity_ned(float *vx, float *vy, float *vz);
int get_gps_satellite_numbers(void);

#endif
