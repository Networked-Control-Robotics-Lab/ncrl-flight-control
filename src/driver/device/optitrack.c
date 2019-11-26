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

void optitrack_handler(uint8_t c)
{
	optitrack_buf_push(c);
	if(c == '+' && optitrack_buf[0] == '@') {
		/* decode optitrack message */
		if(optitrack_serial_decoder(optitrack_buf) == 0) {
			led_on(LED_G);
			optitrack_buf_pos = 0; //reset position pointer
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
#if 0
	float dt = (optitrack.time_now - optitrack.time_last) * 0.001;
#endif

#if 1
	float dt = 1.0f / 50.0f; //fixed dt
#endif
	optitrack.vel_x = (optitrack.pos_x - pos_last.x) / dt;
	optitrack.vel_y = (optitrack.pos_y - pos_last.y) / dt;
	optitrack.vel_z = (optitrack.pos_z - pos_last.z) / dt;
	optitrack.recv_freq = 1.0f / dt;
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

	memcpy(&optitrack.pos_x, &buf[3], sizeof(float));
	memcpy(&optitrack.pos_y, &buf[7], sizeof(float));
	memcpy(&optitrack.pos_z, &buf[11], sizeof(float));
	memcpy(&optitrack.quat_x, &buf[15], sizeof(float));
	memcpy(&optitrack.quat_z, &buf[19], sizeof(float));
	memcpy(&optitrack.quat_y, &buf[23], sizeof(float));
	memcpy(&optitrack.quat_w, &buf[27], sizeof(float));

	if(vel_init_ready == false) {
		optitrack.time_last = get_sys_time_ms();
		pos_last.x = optitrack.pos_x;
		pos_last.y = optitrack.pos_y;
		pos_last.z = optitrack.pos_z;
		optitrack.vel_x = 0.0f;
		optitrack.vel_y = 0.0f;
		optitrack.vel_z = 0.0f;
		vel_init_ready = true;
		return 0;
	}

	optitrack_numerical_vel_calc();
	pos_last.x = optitrack.pos_x; //save for next iteration
	pos_last.y = optitrack.pos_y;
	pos_last.z = optitrack.pos_z;
	optitrack.time_last = optitrack.time_now;

	return 0;
}
