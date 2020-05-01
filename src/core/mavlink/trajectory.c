#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"
#include "sys_time.h"

bool traj_do_ack = false;
uint8_t traj_ack_val;

bool traj_receiving = false;
int traj_list_size = 0;
uint8_t traj_index = 0;

bool z_traj_enabled = false;
bool yaw_traj_enabled = false;

float recept_start_time = 0.0f;
bool recept_finished = false;

bool x_received = false;
bool y_received = false;
bool z_received = false;
bool yaw_received = false;

void polynomial_trajectory_microservice_handler(void)
{
	if(traj_do_ack == true) {
		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_ack_pack_chan(
		        1, 1, MAVLINK_COMM_1, &msg, 255, 0, traj_ack_val);
		send_mavlink_msg_to_uart(&msg);
		traj_do_ack = false;
	}

	if(traj_receiving == true || recept_finished == true) {
		float current_time = get_sys_time_s();
		if((current_time - recept_start_time) >= 5.0f) {
			if(recept_finished == true) {
				/* succeeded: close transaction after 5 seconds in case
				 * the ground station didn't received the ack message */
				recept_finished = false;
			} else {
				/* timeout: transaction failed! */
				//reset autopilot manager
				autopilot_config_trajectory_following(0, false, false);
			}

			//stop receiving trajectory item message
			traj_receiving = false;

			/* reset reception flags */
			z_traj_enabled = false;
			yaw_traj_enabled = false;
			x_received = false;
			y_received = false;
			z_received = false;
			yaw_received = false;
			traj_index = 0;
			traj_list_size = 0;

			//TODO: unlock autopilot
		}
	}
}

void trigger_polynomial_trajectory_ack_sending(uint8_t ack_val)
{
	traj_ack_val = ack_val;
	traj_do_ack = true;
}

void mav_polynomial_trajectory_write(mavlink_message_t *received_msg)
{
	int ack_val;

	//TODO: check if autopilot is busy
	//TODO: lock autopilot
	recept_start_time = get_sys_time_s();

	mavlink_polynomial_trajectory_write_t poly_traj_write;
	mavlink_msg_polynomial_trajectory_write_decode(received_msg, &poly_traj_write);

	z_traj_enabled = (poly_traj_write.z_enabled == 0 ? false : true);
	yaw_traj_enabled = (poly_traj_write.yaw_enabled == 0 ? false : true);
	traj_list_size = poly_traj_write.list_size;

	autopilot_config_trajectory_following(traj_list_size, z_traj_enabled, yaw_traj_enabled);

	x_received = false;
	y_received = false;
	z_received = false;
	yaw_received = false;
	recept_finished = false;
	traj_receiving = true;
	traj_index = 0;

	ack_val = TRAJECTORY_ACK_OK;
	trigger_polynomial_trajectory_ack_sending(ack_val);
}

void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg)
{
	//TODO: check if trajectory exists!

	mavlink_polynomial_trajectory_cmd_t poly_traj_cmd;
	mavlink_msg_polynomial_trajectory_cmd_decode(received_msg, &poly_traj_cmd);

	uint8_t option = poly_traj_cmd.option;

	switch(poly_traj_cmd.cmd) {
	case TRAJECTORY_FOLLOWING_START: {
		bool loop = (option == 0 ? false : true);
		autopilot_trajectory_following_start(loop);
		break;
	}
	case TRAJECTORY_FOLLOWING_STOP:
		autopilot_trajectory_following_stop();
		break;
	}
}

void mav_polynomial_trajectory_item(mavlink_message_t *received_msg)
{
	/* should not receive any trajectory if handshaking not happened */
	if(traj_receiving == false) return;

	recept_start_time = get_sys_time_s();

	mavlink_polynomial_trajectory_item_t poly_traj_item;
	mavlink_msg_polynomial_trajectory_item_decode(received_msg, &poly_traj_item);

	uint8_t ack_val;
	int ret_val = 0;

	/* check current trajectory index to receive */
	if(poly_traj_item.index != traj_index) {
		/* index mismatched */
		return;
	}

	/* save trajectory and decide next receiption type */
	switch(poly_traj_item.type) {
	case TRAJECTORY_POSITION_X:
		ret_val = autopilot_set_x_trajectory(traj_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		x_received = true;
		break;
	case TRAJECTORY_POSITION_Y:
		ret_val = autopilot_set_y_trajectory(traj_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		y_received = true;
		break;
	case TRAJECTORY_POSITION_Z:
		ret_val = autopilot_set_z_trajectory(traj_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		z_received = true;
		break;
	case TRAJECTORY_ANGLE_YAW:
		ret_val = autopilot_set_yaw_trajectory(traj_index, poly_traj_item.coeff,
		                                       poly_traj_item.flight_time);
		yaw_received = true;
		break;
	}

	if((x_received == true) && (y_received == true) &&
	    (z_received == true || z_traj_enabled == false) &&
	    (yaw_received == true || yaw_traj_enabled == false)) {
		traj_index++; //next trajectory segment to receive
		x_received = false;
		y_received = false;
		z_received = false;
		yaw_received = false;
	}

	ack_val = TRAJECTORY_ACK_OK;
	trigger_polynomial_trajectory_ack_sending(ack_val);

	/* finish receiving all trajectory segments */
	if(traj_index >= traj_list_size) {
		recept_finished = true;
		//TODO: unlock autopilot
	}
}
