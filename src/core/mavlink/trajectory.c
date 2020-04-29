#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"
#include "sys_time.h"

uint8_t traj_ack_val;
uint8_t traj_index = 0;
bool traj_do_ack = false;
bool traj_receiving = false;
int traj_list_size = 0;
uint8_t next_traj_type = TRAJECTORY_POSITION_X;
bool z_traj_enabled = false;
bool yaw_traj_enabled = false;
float recept_start_time = 0.0f;

void polynomial_trajectory_microservice_handler(void)
{
	if(traj_do_ack == true) {
		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_ack_pack_chan(
		        1, 1, MAVLINK_COMM_1, &msg, 255, 0, traj_ack_val);
		send_mavlink_msg_to_uart(&msg);
		traj_do_ack = false;
	}

	if(traj_receiving == true) {
		float current_time = get_sys_time_s();
		if((current_time - recept_start_time) >= 5.0f) {
			traj_receiving = false;
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

	traj_receiving = true;
	z_traj_enabled = (poly_traj_write.z_enabled == 0 ? false : true);
	yaw_traj_enabled = (poly_traj_write.yaw_enabled == 0 ? false : true);
	traj_list_size = poly_traj_write.list_size;

	ack_val = TRAJECTORY_ACK_OK;

	trigger_polynomial_trajectory_ack_sending(ack_val);
}

void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg)
{
	mavlink_polynomial_trajectory_cmd_t poly_traj_cmd;
	mavlink_msg_polynomial_trajectory_cmd_decode(received_msg, &poly_traj_cmd);

	uint8_t option = poly_traj_cmd.option;

	switch(poly_traj_cmd.cmd) {
	case TRAJECTORY_FOLLOWING_START: {
		bool loop;
		if(option == 0) {
			loop = false;
		} else {
			loop = true;
		}
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

	/* check current trajectory index/type to receive */
	if(poly_traj_item.index != traj_index ||
	    poly_traj_item.type != next_traj_type) {
		ack_val = TRAJECTORY_ACK_INDEX_MISMATCHED;
		trigger_polynomial_trajectory_ack_sending(ack_val);
		return;
	}

	/* save trajectory and decide next receiption type */
	switch(next_traj_type) {
	case TRAJECTORY_POSITION_X:
		ret_val = autopilot_set_x_trajectory(traj_index, poly_traj_item.coeff);
		next_traj_type = TRAJECTORY_POSITION_Y;
		break;
	case TRAJECTORY_POSITION_Y:
		ret_val = autopilot_set_y_trajectory(traj_index, poly_traj_item.coeff);
		if(z_traj_enabled == true) {
			next_traj_type = TRAJECTORY_POSITION_Z;
		} else {
			if(yaw_traj_enabled == true) {
				next_traj_type = TRAJECTORY_ANGLE_YAW;
			} else {
				next_traj_type = TRAJECTORY_POSITION_X;
			}
		}
		break;
	case TRAJECTORY_POSITION_Z:
		ret_val = autopilot_set_z_trajectory(traj_index, poly_traj_item.coeff);
		if(yaw_traj_enabled == true) {
			next_traj_type = TRAJECTORY_ANGLE_YAW;
		} else {
			next_traj_type = TRAJECTORY_POSITION_X;
		}
		break;
	case TRAJECTORY_ANGLE_YAW:
		ret_val = autopilot_set_yaw_trajectory(traj_index, poly_traj_item.coeff);
		next_traj_type = TRAJECTORY_POSITION_X;
		break;
	}

	traj_index++; //next trajectory segment to receive

	/* trigger sending ack message */
	if(ret_val == AUTOPILOT_SET_SUCCEED) {
		ack_val = TRAJECTORY_ACK_OK;
	} else {
		ack_val = TRAJECTORY_ACK_ERROR; //TODO: more error codes?
	}
	trigger_polynomial_trajectory_ack_sending(ack_val);

	/* finish receiving all trajectory segments */
	if(traj_index == traj_list_size) {
		traj_receiving = false;
		traj_list_size = 0;
		//TODO: unlock autopilot
	}
}
