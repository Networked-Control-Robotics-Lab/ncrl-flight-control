#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "autopilot.h"
#include "sys_time.h"
#include "optitrack.h"
#include "system_state.h"
#include "../mavlink/mav_publisher.h"
#include "../mavlink/mav_trajectory.h"
#include "sys_param.h"
#include "common_list.h"
#include "trajectory_following.h"

traj_msg_manager_t traj_msg_manager;

void polynomial_trajectory_microservice_handler(void)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	if(traj_msg_manager.do_ack == true) {
		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_ack_pack_chan(
		        (uint8_t)sys_id, 1, MAVLINK_COMM_1, &msg, 255, 0, traj_msg_manager.ack_val);
		send_mavlink_msg_to_uart(&msg);
		traj_msg_manager.do_ack = false;
	}

	if(traj_msg_manager.do_recept == true || traj_msg_manager.recept_finished == true) {
		float current_time = get_sys_time_s();
		if((current_time - traj_msg_manager.recept_start_time) >= 5.0f) {
			if(traj_msg_manager.recept_finished == true) {
				/* succeeded: close transaction after 5 seconds in case
				 * the ground station didn't received the ack message */
				traj_msg_manager.recept_finished = false;
			} else {
				/* timeout: transaction failed! */
				//reset autopilot manager
				autopilot_config_trajectory_following(0, false, false);
			}

			//stop receiving trajectory item message
			traj_msg_manager.do_recept = false;

			/* reset reception flags */
			traj_msg_manager.z_planned = false;
			traj_msg_manager.yaw_planned = false;
			traj_msg_manager.x_recvd = false;
			traj_msg_manager.y_recvd = false;
			traj_msg_manager.z_recvd = false;
			traj_msg_manager.yaw_recvd = false;
			traj_msg_manager.recept_index = 0;
			traj_msg_manager.list_size = 0;
		}
	}
}

void trigger_polynomial_trajectory_ack_sending(uint8_t ack_val)
{
	traj_msg_manager.ack_val = ack_val;
	traj_msg_manager.do_ack = true;
}

void mav_polynomial_trajectory_write(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode polynomial_trajectory_write message */
	mavlink_polynomial_trajectory_write_t poly_traj_write;
	mavlink_msg_polynomial_trajectory_write_decode(received_msg, &poly_traj_write);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != poly_traj_write.target_system) {
		return;
	}

	traj_msg_manager.recept_start_time = get_sys_time_s();

	/* setup trajectory configuration of autopilot*/
	traj_msg_manager.z_planned = (poly_traj_write.z_enabled == 0 ? false : true);
	traj_msg_manager.yaw_planned = (poly_traj_write.yaw_enabled == 0 ? false : true);
	traj_msg_manager.list_size = poly_traj_write.list_size;
	int ret_val = autopilot_config_trajectory_following(traj_msg_manager.list_size, traj_msg_manager.z_planned,
	                traj_msg_manager.yaw_planned);

	switch(ret_val) {
	case AUTOPILOT_TRAJACTORY_LIST_TOO_LARGE:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_TOO_LARGE);
		break;
	case AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_BUSY);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
		break;
	default:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_ERROR);
	}

	traj_msg_manager.x_recvd = false;
	traj_msg_manager.y_recvd = false;
	traj_msg_manager.z_recvd = false;
	traj_msg_manager.yaw_recvd = false;
	traj_msg_manager.recept_finished = false;
	traj_msg_manager.do_recept = true;
	traj_msg_manager.recept_index = 0;
}

void mav_polynomial_trajectory_cmd(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode polynomial_trajectory_cmd message */
	mavlink_polynomial_trajectory_cmd_t poly_traj_cmd;
	mavlink_msg_polynomial_trajectory_cmd_decode(received_msg, &poly_traj_cmd);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != poly_traj_cmd.target_system) {
		return;
	}

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
	case AUTOPILOT_TRAJACTORY_LIST_EMPTY:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_EMPTY);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
		break;
	default:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_ERROR);
	}
}

void mav_polynomial_trajectory_item(mavlink_message_t *received_msg)
{
	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	/* decode polynomial_trajectory_item message */
	mavlink_polynomial_trajectory_item_t poly_traj_item;
	mavlink_msg_polynomial_trajectory_item_decode(received_msg, &poly_traj_item);

	/* ignore the message if the target id not matched to the system id */
	if((uint8_t)sys_id != poly_traj_item.target_system) {
		return;
	}

	/* should not receive any trajectory if handshaking not happened */
	if(traj_msg_manager.do_recept == false) return;

	traj_msg_manager.recept_start_time = get_sys_time_s();

	int ret_val = 0;

	/* check current trajectory index to receive */
	if(poly_traj_item.index != traj_msg_manager.recept_index) {
		/* index mismatched */
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_ERROR);
		return;
	}

	/* save trajectory and decide next receiption type */
	switch(poly_traj_item.type) {
	case TRAJECTORY_POSITION_X:
		ret_val = autopilot_set_x_trajectory(traj_msg_manager.recept_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		traj_msg_manager.x_recvd = true;
		break;
	case TRAJECTORY_POSITION_Y:
		ret_val = autopilot_set_y_trajectory(traj_msg_manager.recept_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		traj_msg_manager.y_recvd = true;
		break;
	case TRAJECTORY_POSITION_Z:
		ret_val = autopilot_set_z_trajectory(traj_msg_manager.recept_index, poly_traj_item.coeff,
		                                     poly_traj_item.flight_time);
		traj_msg_manager.z_recvd = true;
		break;
	case TRAJECTORY_ANGLE_YAW:
		ret_val = autopilot_set_yaw_trajectory(traj_msg_manager.recept_index, poly_traj_item.coeff,
		                                       poly_traj_item.flight_time);
		traj_msg_manager.yaw_recvd = true;
		break;
	}

	switch(ret_val) {
	case AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_BUSY);
		break;
	case AUTOPILOT_TRAJACTORY_LIST_FULL:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_LIST_FULL);
		break;
	case AUTOPILOT_SET_SUCCEED:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_OK);
		break;
	default:
		trigger_polynomial_trajectory_ack_sending(TRAJECTORY_ACK_ERROR);
	}

	if((traj_msg_manager.x_recvd == true) && (traj_msg_manager.y_recvd == true) &&
	    (traj_msg_manager.z_recvd == true || traj_msg_manager.z_planned == false) &&
	    (traj_msg_manager.yaw_recvd == true || traj_msg_manager.yaw_planned == false)) {
		traj_msg_manager.recept_index++; //next trajectory segment to receive
		traj_msg_manager.x_recvd = false;
		traj_msg_manager.y_recvd = false;
		traj_msg_manager.z_recvd = false;
		traj_msg_manager.yaw_recvd = false;
	}

	/* finish receiving all trajectory segments */
	if(traj_msg_manager.recept_index >= traj_msg_manager.list_size) {
		traj_msg_manager.recept_finished = true;
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_position_debug_pack(
	        (uint8_t)sys_id, 1, &msg, target_system, target_component,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_velocity_debug_pack(
	        (uint8_t)sys_id, 1, &msg, target_system, target_component,
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

	float sys_id;
	get_sys_param_float(MAV_SYS_ID, &sys_id);

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_acceleration_debug_pack(
	        (uint8_t)sys_id, 1, &msg, target_system, target_component,
	        des_accel_ff[0], des_accel_ff[1], des_accel_ff[2]);
	send_mavlink_msg_to_uart(&msg);
}
