#ifndef __MAV_TRAJECTORY_H__
#define __MAV_TRAJECTORY_H__

#include "mavlink.h"

typedef struct {
	bool do_ack;
	uint8_t ack_val;

	bool do_recept;
	int list_size;
	uint8_t recept_index;
	float recept_start_time;
	bool recept_finished;

	bool z_planned;
	bool yaw_planned;

	bool x_recvd;
	bool y_recvd;
	bool z_recvd;
	bool yaw_recvd;
} traj_msg_manager_t;

void mav_polynomial_trajectory_write(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg);
void mav_polynomial_trajectory_item(mavlink_message_t *received_msg);

void send_mavlink_trajectory_position_debug(void);
void send_mavlink_trajectory_velocity_debug(void);
void send_mavlink_trajectory_acceleration_debug(void);

void polynomial_trajectory_microservice_handler(void);

#endif
