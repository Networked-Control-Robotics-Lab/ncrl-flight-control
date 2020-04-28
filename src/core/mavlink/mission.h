#ifndef __MISSION_H__
#define __MISSION_H__

#include "mavlink.h"

void mav_command_long(mavlink_message_t *received_msg);
void mav_mission_request_list(mavlink_message_t *received_msg);
void mav_position_target_local_ned(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_write(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_item(mavlink_message_t *received_msg);

void mission_waypoint_ack_handler(void);
void polynomial_trajectory_ack_handler(void);

#endif
