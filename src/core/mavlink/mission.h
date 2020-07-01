#ifndef __MISSION_H__
#define __MISSION_H__

#include "mavlink.h"

typedef struct {
	/* transmission */
	bool send_mission = false;
	float sender_timout_timer = 0.0f;
      
	/* reception */
	bool receive_mission = false;
	int recept_mission_cnt = 0;
	int recept_mission_index = 0;
	int recvd_mission_type;
	float recept_timout_timer = 0.0f; 
	int recept_retry = 0.0f;
} mavlink_mission_manager;

void mav_command_long(mavlink_message_t *received_msg);
void mav_mission_request_list(mavlink_message_t *received_msg);
void mav_mission_count(mavlink_message_t *received_msg);
void mav_mission_item_int(mavlink_message_t *received_msg);
void mav_mission_request_int(mavlink_message_t *received_msg);
void mav_mission_ack(mavlink_message_t *received_msg);
void mav_mission_clear_all(mavlink_message_t *received_msg);

void mission_waypoint_microservice_handler(void);

#endif
