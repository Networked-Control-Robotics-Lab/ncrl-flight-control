#ifndef __COMPASS_H__
#define __COMPASS_H__

#include "debug_link.h"

bool is_compass_available(void);
void compass_wait_until_stable(void);
void get_compass_raw(float *mag);
void get_compass_lpf(float *mag);
float get_compass_raw_strength(void);
float get_compass_lpf_strength(void);
float get_compass_update_rate(void);

void send_compass_debug_message(debug_msg_t *payload);

#endif

