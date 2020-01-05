#include "./parser.h"

#if 0
/* enumerate mavlink handler function */
enum ENUM_MAV_CMDS {
	ENUM_HANDLER_FUNC(mission_read_waypoint_list),
	ENUM_HANDLER_FUNC(mission_write_waypoint_list),
	ENUM_HANDLER_FUNC(mission_clear_waypoint),
	ENUM_HANDLER_FUNC(mission_set_new_current_waypoint),
	ENUM_HANDLER_FUNC(parameter_read_value),
	ENUM_HANDLER_FUNC(parameter_read_single_value),
	ENUM_HANDLER_FUNC(parameter_write_value),
	MAV_CMD_CNT
};

/* register mavlink msg id to the handler function */
struct mavlink_parser_item cmd_list[] = {
	//mission commands
	MAV_CMD_DEF(mission_read_waypoint_list, 43),
	MAV_CMD_DEF(mission_write_waypoint_list, 44),
	MAV_CMD_DEF(mission_clear_waypoint, 45),
	MAV_CMD_DEF(mission_set_new_current_waypoint, 41),
	MAV_CMD_DEF(mission_command, 76),
	//onboard parameter access commands
	MAV_CMD_DEF(parameter_read_value, 21),
	MAV_CMD_DEF(parameter_read_single_value, 20),
	MAV_CMD_DEF(parameter_write_value, 23)
};

void parse_mavlink_received_msg(mavlink_message_t *msg)
{
	int i;
	for(i = 0; i < (signed int)CMD_LEN(cmd_list); i++) {
		if(msg->msgid == cmd_list[i].msgid) {
			cmd_list[i].handler();
			//clear_message_id(msg);
			break;
		}
	}
}
#endif
