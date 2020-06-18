#ifndef __PARAM_H__
#define __PARAM_H__

#include "mavlink.h"

void mav_param_request_list(mavlink_message_t *received_msg);
void mav_param_request_read(mavlink_message_t *received_msg);
void mav_param_set(mavlink_message_t *received_msg);

#endif
