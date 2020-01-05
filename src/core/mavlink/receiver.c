#include "stm32f4xx.h"
#include "mavlink.h"

mavlink_message_t mavlink_recpt_msg;
mavlink_status_t mavlink_recpt_status;

void mavlink_serial_recept_handler(uint8_t buf)
{
	mavlink_parse_char(MAVLINK_COMM_0, buf, &mavlink_recpt_msg, &mavlink_recpt_status);
}
