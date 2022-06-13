#ifndef __MAV_MISSION_H__
#define __MAV_MISSION_H__

#include <stdint.h>
#include <stdbool.h>
#include "mavlink.h"

typedef struct {
	/* transmission */
	bool send_mission;
	float sender_timout_timer;

	/* reception */
	bool receive_mission;
	int recept_cnt;
	int recept_index;
	int recvd_mission_type;
	float recept_timout_timer;
	int recept_retry;
} mavlink_mission_manager;

void mav_set_mode(mavlink_message_t *received_msg);

void mav_mission_request_list(mavlink_message_t *received_msg);
void mav_mission_count(mavlink_message_t *received_msg);
void mav_mission_item_int(mavlink_message_t *received_msg);
void mav_mission_request_int(mavlink_message_t *received_msg);
void mav_mission_ack(mavlink_message_t *received_msg);
void mav_mission_clear_all(mavlink_message_t *received_msg);

void mission_waypoint_microservice_handler(void);

#endif
