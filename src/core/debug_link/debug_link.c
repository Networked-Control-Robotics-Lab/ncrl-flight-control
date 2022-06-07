#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "debug_link.h"
#include "proj_config.h"
#include "board_porting.h"

void pack_debug_debug_message_header(debug_msg_t *payload, int message_id)
{
	payload->len = 3; //reserve for header debug_message
	payload->s[2] = message_id;
}

void pack_debug_debug_message_float(float *data_float, debug_msg_t *payload)
{
	memcpy((uint8_t *)&payload->s + payload->len, (uint8_t *)data_float, sizeof(float));
	payload->len += sizeof(float);
}

void pack_debug_debug_message_int32(int32_t *data_int32, debug_msg_t *payload)
{
	memcpy((uint8_t *)&payload->s + payload->len, (uint8_t *)data_int32, sizeof(int32_t));
	payload->len += sizeof(float);
}

static uint8_t generate_debug_debug_message_checksum(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

void send_onboard_data(uint8_t *payload, int payload_count)
{
	uint8_t checksum;

	checksum = generate_debug_debug_message_checksum(payload + 3, payload_count - 3);

	payload[0] = '@';
	payload[1] = payload_count - 3;

	payload[payload_count] = checksum;
	payload_count++;

	debug_link_puts((char *)payload, payload_count);
}

void send_general_float_debug_message(float val, debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_GENERAL_FLOAT);
	pack_debug_debug_message_float(&val, payload);
}
