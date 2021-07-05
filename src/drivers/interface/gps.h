#ifndef __GPS_H__
#define __GPS_H__

bool is_gps_available(void);
void get_gps_longitude_latitude_height_s32(int32_t *longitude, int32_t *latitude, int32_t *height_msl);
void get_gps_velocity_ned(float *vx, float *vy, float *vz);
int get_gps_satellite_numbers(void);
void get_gps_dilution_of_precision(float *pdop, float *hdop, float *vdop);
uint8_t get_gps_fix_type(void);
void get_gps_position_uncertainty(float *h_acc, float *v_acc);
float get_gps_ground_speed(void);
float get_gps_heading(void);
float get_gps_update_freq(void);
float get_gps_last_update_time_ms(void);

#endif
