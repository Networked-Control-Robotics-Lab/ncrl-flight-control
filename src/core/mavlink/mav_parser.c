#include "../../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/mav_parser.h"
#include "../mavlink/mav_mission.h"
#include "../mavlink/mav_param.h"
#include "../mavlink/mav_trajectory.h"
#include "../mavlink/mav_command.h"

/* enumerate mavlink handler function */
enum ENUM_MAV_CMDS {
	/* common mavlink messages */
	ENUM_HANDLER_FUNC(mav_set_mode),
	ENUM_HANDLER_FUNC(mav_mission_request_list),
	ENUM_HANDLER_FUNC(mav_mission_count),
	ENUM_HANDLER_FUNC(mav_mission_item_int),
	ENUM_HANDLER_FUNC(mav_mission_request_int),
	ENUM_HANDLER_FUNC(mav_mission_ack),
	ENUM_HANDLER_FUNC(mav_mission_clear_all),
	//ENUM_HANDLER_FUNC(mav_mission_set_current),
	ENUM_HANDLER_FUNC(mav_command_long),
	ENUM_HANDLER_FUNC(mav_param_request_list),
	ENUM_HANDLER_FUNC(mav_param_request_read),
	ENUM_HANDLER_FUNC(mav_param_set),
	/* extended mavlink messages */
	ENUM_HANDLER_FUNC(mav_polynomial_trajectory_write),
	ENUM_HANDLER_FUNC(mav_polynomial_trajectory_cmd),
	ENUM_HANDLER_FUNC(mav_polynomial_trajectory_item),
	MAV_CMD_CNT
};

/* register mavlink msg id to the handler function */
struct mavlink_parser_item cmd_list[] = {
	/* common mavlink messages */
	MAV_CMD_DEF(mav_set_mode, 11),
	MAV_CMD_DEF(mav_mission_request_list, 43),
	MAV_CMD_DEF(mav_mission_count, 44),
	MAV_CMD_DEF(mav_mission_item_int, 73),
	MAV_CMD_DEF(mav_mission_request_int, 40),
	MAV_CMD_DEF(mav_mission_ack, 47),
	MAV_CMD_DEF(mav_mission_clear_all, 45),
	//MAV_CMD_DEF(mav_mission_set_current, 41),
	MAV_CMD_DEF(mav_command_long, 76),
	MAV_CMD_DEF(mav_param_request_list, 21),
	MAV_CMD_DEF(mav_param_request_read, 20),
	MAV_CMD_DEF(mav_param_set, 23),
	/* extended mavlink messages */
	MAV_CMD_DEF(mav_polynomial_trajectory_write, 11000),
	MAV_CMD_DEF(mav_polynomial_trajectory_cmd, 11001),
	MAV_CMD_DEF(mav_polynomial_trajectory_item, 11003)
};

void parse_mavlink_received_msg(mavlink_message_t *msg)
{
	int i;
	for(i = 0; i < (signed int)CMD_LEN(cmd_list); i++) {
		if(msg->msgid == cmd_list[i].msg_id) {
			cmd_list[i].handler(msg);
			break;
		}
	}
}
