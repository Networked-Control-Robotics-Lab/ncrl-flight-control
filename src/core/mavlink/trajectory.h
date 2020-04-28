#ifndef __MAVLINK_TRAJECTORY_H__
#define __MAVLINK_TRAJECTORY_H__

#include "mavlink.h"

void mav_polynomial_trajectory_write(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_item(mavlink_message_t *received_msg);

#endif
