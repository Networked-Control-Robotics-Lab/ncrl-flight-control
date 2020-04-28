#include "mavlink.h"
#include "ncrl.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"

uint8_t traj_ack_val;
bool traj_do_ack = false;

void polynomial_trajectory_ack_handler(void)
{
	if(traj_do_ack == true) {
		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_ack_pack_chan(
		        1, 1, MAVLINK_COMM_1, &msg, 255, 0, traj_ack_val);
		send_mavlink_msg_to_uart(&msg);
		traj_do_ack = false;
	}
}

void trigger_polynomial_trajectory_ack_sending(uint8_t ack_val)
{
	traj_ack_val = ack_val;
	traj_do_ack = true;
}

void mav_polynomial_trajectory_write(mavlink_message_t *received_msg)
{
	//uint8_t list_size;
	mavlink_polynomial_trajectory_write_t poly_traj_write;
	mavlink_msg_polynomial_trajectory_write_decode(received_msg, &poly_traj_write);
	//list_size = poly_traj_write.list_size;
}

void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg)
{
	mavlink_polynomial_trajectory_cmd_t poly_traj_cmd;
	mavlink_msg_polynomial_trajectory_cmd_decode(received_msg, &poly_traj_cmd);

	//bool option = poly_traj_cmd.option;

	switch(poly_traj_cmd.cmd) {
	case TRAJECTORY_FOLLOWING_START:
		break;
	case TRAJECTORY_FOLLOWING_STOP:
		break;
	}
}

void mav_polynomial_trajectory_item(mavlink_message_t *received_msg)
{
	mavlink_polynomial_trajectory_item_t poly_traj_item;
	mavlink_msg_polynomial_trajectory_item_decode(received_msg, &poly_traj_item);

	int index = 0; //TODO: trajectory index

	int ret_val = autopilot_add_write_trajectory(index, poly_traj_item.x_coeff,
	                poly_traj_item.y_coeff,
	                poly_traj_item.z_coeff,
	                poly_traj_item.yaw_coeff);

	uint8_t ack_val;
	if(ret_val == AUTOPILOT_SET_SUCCEED) {
		ack_val = TRAJECTORY_ACK_OK;
	} else {
		ack_val = TRAJECTORY_ACK_ERROR; //TODO: more error codes?
	}
	trigger_polynomial_trajectory_ack_sending(ack_val);
}
