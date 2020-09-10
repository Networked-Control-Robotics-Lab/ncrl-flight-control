#ifndef __COMPASS_H__
#define __COMPASS_H__

bool is_compass_present(void);
void get_imu_compass_raw(float *mag_raw);
float get_imu_compass_raw_strength(void);
float get_imu_compass_update_freq(void);

void send_compass_debug_message(debug_msg_t *payload);

#endif
