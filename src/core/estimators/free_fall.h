#ifndef __FREE_FALL_H__
#define __FREE_FALL_H__

bool free_fall_detect(float *g, float *g_norm);
void send_free_fall_debug_message(debug_msg_t *payload);

#endif
