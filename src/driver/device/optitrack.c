#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "uart.h"
#include "optitrack.h"

#define OPTITRACK_SERIAL_MSG_SIZE 30

uint8_t optitrack_msg_buf[OPTITRACK_SERIAL_MSG_SIZE] = {0};
volatile int received_cnt = 0;

optitrack_t optitrack;

void optitrack_handler(uint8_t c)
{
#if 0   /* debug print */
	uart_putc(USART3, c);
	return;
#endif
	if(received_cnt >= OPTITRACK_SERIAL_MSG_SIZE) {
		received_cnt = 0;
	}

	optitrack_msg_buf[received_cnt] = c;
	received_cnt++;

	/* parse the message if next start byte is received */
	if(c == '@') { //XXX:replace decode condition with a message end byte test?
		if(optitrack_msg_buf[0] == '@' && received_cnt == OPTITRACK_SERIAL_MSG_SIZE) {
			//optitrack_serial_decoder(&optitrack_msg_buf[0]);
		}

		optitrack_msg_buf[0] = '@';
		received_cnt = 1;
	}
}

#define OPTITRACK_CHECKSUM_INIT_VAL 19
static uint8_t generate_optitrack_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = OPTITRACK_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

void optitrack_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
#if 1
	memcpy(&optitrack.pos_x, &buf[2], sizeof(float));
	memcpy(&optitrack.pos_y, &buf[6], sizeof(float));
	memcpy(&optitrack.pos_z, &buf[10], sizeof(float));
	memcpy(&optitrack.quat_x, &buf[14], sizeof(float));
	memcpy(&optitrack.quat_z, &buf[18], sizeof(float));
	memcpy(&optitrack.quat_y, &buf[22], sizeof(float));
	memcpy(&optitrack.quat_w, &buf[26], sizeof(float));

	uint8_t checksum = generate_optitrack_checksum_byte(&buf[2], OPTITRACK_SERIAL_MSG_SIZE - 2);
	if(checksum != recv_checksum) {
		/* TODO */
	}
#endif
}
