#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.h"
#include "autopilot.h"
#include "sys_time.h"
#include "optitrack.h"
#include "localization_system.h"

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
	recept_start_time = get_sys_time_s();

	mavlink_polynomial_trajectory_write_t poly_traj_write;
	mavlink_msg_polynomial_trajectory_write_decode(received_msg, &poly_traj_write);

	/* setup trajectory configuration of autopilot*/
	z_traj_enabled = (poly_traj_write.z_enabled == 0 ? false : true);
	yaw_traj_enabled = (poly_traj_write.yaw_enabled == 0 ? false : true);
	traj_list_size = poly_traj_write.list_size;
	int ret_val = autopilot_config_trajectory_following(traj_list_size, z_traj_enabled,
	                yaw_traj_enabled);

	switch(ret_val) {
	case AUTOPILOT_TRAJ_LIST_FULL:
		//TODO
		//trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_FULL);
		break;
	case AUTOPILOT_TRAJ_EXECUTING:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_BUSY);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
		break;
	}

	x_received = false;
	y_received = false;
	z_received = false;
	yaw_received = false;
	recept_finished = false;
	traj_receiving = true;
	traj_index = 0;
}

void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg)
{
	mavlink_polynomial_trajectory_cmd_t poly_traj_cmd;
	mavlink_msg_polynomial_trajectory_cmd_decode(received_msg, &poly_traj_cmd);
	uint8_t option = poly_traj_cmd.option;

	int ret_val = AUTOPILOT_SET_SUCCEED;

	/* change autopilot's mode */
	switch(poly_traj_cmd.cmd) {
	case TRAJECTORY_FOLLOWING_START: {
		bool loop = (option == 0 ? false : true);
		ret_val = autopilot_trajectory_following_start(loop);
		break;
	}
	case TRAJECTORY_FOLLOWING_STOP:
		ret_val = autopilot_trajectory_following_stop();
		break;
	}

	/* send trajectory ack message */
	switch(ret_val) {
	case AUTOPILOT_TRAJ_EXECUTING:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_BUSY);
		break;
	case AUTOPILOT_TRAJ_LIST_EMPTY:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_EMPTY);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
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

	int ret_val = 0;

	/* check current trajectory index to receive */
	if(poly_traj_item.index != traj_index) {
		/* index mismatched */
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_ERROR);
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

	switch(ret_val) {
	case AUTOPILOT_TRAJ_EXECUTING:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_BUSY);
		break;
	case AUTOPILOT_TRAJ_LIST_FULL:
		//TODO
		//trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_FULL);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
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

	/* finish receiving all trajectory segments */
	if(traj_index >= traj_list_size) {
		recept_finished = true;
	}
}

void send_mavlink_trajectory_position_debug(void)
{
	uint8_t target_system = 0;
	uint8_t target_component = 0;

	float curr_pos[3] = {0.0f};
	float des_pos[3] = {0.0f};

	get_enu_position(curr_pos);
	autopilot_get_pos_setpoint(des_pos);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_position_debug_pack(
	        1, 1, &msg, target_system, target_component,
	        curr_pos[0], curr_pos[1], curr_pos[2],
	        des_pos[0], des_pos[1], des_pos[2]);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_trajectory_velocity_debug(void)
{
	uint8_t target_system = 0;
	uint8_t target_component = 0;

	float curr_vel[3] = {0.0f};
	float des_vel[3] = {0.0f};

	get_enu_velocity(curr_vel);
	autopilot_get_vel_setpoint(des_vel);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_velocity_debug_pack(
	        1, 1, &msg, target_system, target_component,
	        curr_vel[0], curr_vel[1], curr_vel[2],
	        des_vel[0], des_vel[1], des_vel[2]);
	send_mavlink_msg_to_uart(&msg);
}

void send_mavlink_trajectory_acceleration_debug(void)
{
	uint8_t target_system = 0;
	uint8_t target_component = 0;

	float des_accel_ff[3] = {0.0f};

	autopilot_get_accel_feedforward(des_accel_ff);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_acceleration_debug_pack(
	        1, 1, &msg, target_system, target_component,
	        des_accel_ff[0], des_accel_ff[1], des_accel_ff[2]);
	send_mavlink_msg_to_uart(&msg);
}
