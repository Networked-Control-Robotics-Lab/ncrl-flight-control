#ifndef __GPS_TO_ENU_H__
#define __GPS_TO_ENU_H__

bool gps_home_is_set(void);

void set_home_longitude_latitude(float longitude, float latitude, float height_msl);
void get_home_longitude_latitude(float *longitude, float *latitude);
void longitude_latitude_to_enu(float longitude, float latitude, float height_msl,
                               float *x_enu, float *y_enu, float *z_enu);
#endif
