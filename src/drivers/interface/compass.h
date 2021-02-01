#ifndef __COMPASS_H__
#define __COMPASS_H__

#include "debug_link.h"

bool is_compass_present(void);
void get_compass_raw(float *mag);
float get_compass_raw_strength(void);
float get_compass_update_rate(void);
void compass_undistortion(float *mag);

void send_compass_debug_message(debug_msg_t *payload);

#endif

