#ifndef __PARAM_H__
#define __PARAM_H__

#include "mavlink.h"

void mav_param_request_list(mavlink_message_t *received_msg);

void parameter_ack_handler(void);

#endif
