#include "mavlink.h"
#include "../mavlink/parser.h"
#include "../mavlink/mission.h"
#include "../mavlink/param.h"

/* enumerate mavlink handler function */
enum ENUM_MAV_CMDS {
	ENUM_HANDLER_FUNC(mav_mission_request_list),
	//ENUM_HANDLER_FUNC(mav_mission_count),
	//ENUM_HANDLER_FUNC(mav_mission_clear_all),
	//ENUM_HANDLER_FUNC(mav_mission_set_current),
	ENUM_HANDLER_FUNC(mav_set_position_target_local_ned),
	ENUM_HANDLER_FUNC(mav_command_long),
	ENUM_HANDLER_FUNC(mav_param_request_list),
	//ENUM_HANDLER_FUNC(mav_param_request_read),
	//ENUM_HANDLER_FUNC(mav_param_set),
	MAV_CMD_CNT
};

/* register mavlink msg id to the handler function */
struct mavlink_parser_item cmd_list[] = {
	MAV_CMD_DEF(mav_mission_request_list, 43),
	//MAV_CMD_DEF(mav_mission_count, 44),
	//MAV_CMD_DEF(mav_mission_clear_all, 45),
	//MAV_CMD_DEF(mav_mission_set_current, 41),
	MAV_CMD_DEF(mav_set_position_target_local_ned, 84),
	MAV_CMD_DEF(mav_command_long, 76),
	MAV_CMD_DEF(mav_param_request_list, 21),
	//MAV_CMD_DEF(mav_param_request_read, 20),
	//MAV_CMD_DEF(mav_param_set, 23)
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
