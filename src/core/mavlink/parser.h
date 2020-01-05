#ifndef __MAVLINK_PARSER_H__
#define __MAVLINK_PARSER_H__

#include <stdint.h>

#define CMD_LEN(list) (sizeof(list) / sizeof(struct mavlink_cmd))
#define MAV_CMD_DEF(handler_function, id) \
	[handler_function ## _ID] = {.cmd_handler = handler_function, .msgid = id}
#define ENUM_HANDLER_FUNC(handler_function) handler_function ## _ID

struct mavlink_parser_item {
	uint8_t msg_id;
	void (*handler)(void);
};

#endif
