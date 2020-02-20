#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "uart.h"
#include "led.h"
#include "optitrack.h"
#include "vector.h"
#include "sys_time.h"
#include "lpf.h"
#include "debug_link.h"

#define OPTITRACK_SERIAL_MSG_SIZE 32

volatile int optitrack_buf_pos = 0;
uint8_t optitrack_buf[OPTITRACK_SERIAL_MSG_SIZE] = {0};

optitrack_t optitrack;
vector3d_f_t pos_last;
bool vel_init_ready = false;

void optitrack_init(int id)
{
	optitrack.id = id;
}

bool optitrack_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - optitrack.time_now) > 300) {
		led_off(LED_G);
		return false;
	}
	return true;
}

void optitrack_buf_push(uint8_t c)
{
	if(optitrack_buf_pos >= OPTITRACK_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < OPTITRACK_SERIAL_MSG_SIZE; i++) {
			optitrack_buf[i - 1] = optitrack_buf[i];
		}

		/* save new byte to the last array element */
		optitrack_buf[OPTITRACK_SERIAL_MSG_SIZE - 1] = c;
		optitrack_buf_pos = OPTITRACK_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		optitrack_buf[optitrack_buf_pos] = c;
		optitrack_buf_pos++;
	}
}

void optitrack_handler(void)
{
	/* receive a byte of optitrack packet */
	char c;
	while(uart7_getc(&c) == true) {
		optitrack_buf_push(c); //push received byte to the list

		/* decode the packet */
		if(c == '+' && optitrack_buf[0] == '@') {
			/* decode optitrack message */
			if(optitrack_serial_decoder(optitrack_buf) == 0) {
				led_on(LED_G);
				optitrack_buf_pos = 0; //reset position pointer
			}
		}
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

void optitrack_numerical_vel_calc(void)
{
	const float dt = 1.0f / 30.0f; //fixed dt (30Hz)
	optitrack.vel_raw_x = (optitrack.pos_x - pos_last.x) / dt;
	optitrack.vel_raw_y = (optitrack.pos_y - pos_last.y) / dt;
	optitrack.vel_raw_z = (optitrack.pos_z - pos_last.z) / dt;

	float received_period = (optitrack.time_now - optitrack.time_last) * 0.001;
	optitrack.recv_freq = 1.0f / received_period;

	lpf(optitrack.vel_raw_x, &(optitrack.vel_lpf_x), 0.45);
	lpf(optitrack.vel_raw_y, &(optitrack.vel_lpf_y), 0.45);
	lpf(optitrack.vel_raw_z, &(optitrack.vel_lpf_z), 0.45);
}

int optitrack_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_optitrack_checksum_byte(&buf[3], OPTITRACK_SERIAL_MSG_SIZE - 4);
	int recv_id = buf[2];
	if(checksum != recv_checksum || optitrack.id != recv_id) {
		return 1; //error detected
	}

	optitrack.time_now = get_sys_time_ms();

	float ned_pos_x, ned_pos_y, ned_pos_z;

	memcpy(&ned_pos_x, &buf[3], sizeof(float)); //in ned coordinate system
	memcpy(&ned_pos_y, &buf[7], sizeof(float));
	memcpy(&ned_pos_z, &buf[11], sizeof(float));
	optitrack.pos_x = ned_pos_x;
	optitrack.pos_y = ned_pos_y;
	optitrack.pos_z = -ned_pos_z;
	memcpy(&optitrack.q[1], &buf[15], sizeof(float)); //in ned coordinate system
	memcpy(&optitrack.q[2], &buf[19], sizeof(float));
	memcpy(&optitrack.q[3], &buf[23], sizeof(float));
	memcpy(&optitrack.q[0], &buf[27], sizeof(float));

	if(vel_init_ready == false) {
		optitrack.time_last = get_sys_time_ms();
		pos_last.x = optitrack.pos_x;
		pos_last.y = optitrack.pos_y;
		pos_last.z = optitrack.pos_z;
		optitrack.vel_raw_x = 0.0f;
		optitrack.vel_raw_y = 0.0f;
		optitrack.vel_raw_z = 0.0f;
		vel_init_ready = true;
		return 0;
	}

	static int vel_calc_counter = 0;
	if((vel_calc_counter++) == 1) {
		optitrack_numerical_vel_calc();
		pos_last.x = optitrack.pos_x; //save for next iteration
		pos_last.y = optitrack.pos_y;
		pos_last.z = optitrack.pos_z;
		optitrack.time_last = optitrack.time_now;
		vel_calc_counter = 0;
	}

	return 0;
}

void send_optitrack_position_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_POSITION);
	pack_debug_debug_message_float(&optitrack.pos_x, payload);
	pack_debug_debug_message_float(&optitrack.pos_y, payload);
	pack_debug_debug_message_float(&optitrack.pos_z, payload);
}

void send_optitrack_quaternion_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_QUATERNION);
	pack_debug_debug_message_float(&optitrack.q[0], payload);
	pack_debug_debug_message_float(&optitrack.q[1], payload);
	pack_debug_debug_message_float(&optitrack.q[2], payload);
	pack_debug_debug_message_float(&optitrack.q[3], payload);
}

void send_optitrack_velocity_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_VELOCITY);
	pack_debug_debug_message_float(&optitrack.vel_raw_x, payload);
	pack_debug_debug_message_float(&optitrack.vel_raw_y, payload);
	pack_debug_debug_message_float(&optitrack.vel_raw_z, payload);
	pack_debug_debug_message_float(&optitrack.vel_lpf_x, payload);
	pack_debug_debug_message_float(&optitrack.vel_lpf_y, payload);
	pack_debug_debug_message_float(&optitrack.vel_lpf_z, payload);
}
