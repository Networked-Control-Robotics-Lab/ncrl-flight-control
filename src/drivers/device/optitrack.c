#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "gpio.h"
#include "optitrack.h"
#include "sys_time.h"
#include "lpf.h"
#include "debug_link.h"
#include "led.h"
#include "proj_config.h"

#define OPTITRACK_QUEUE_SIZE (32 * 400) //~400 packets
#define OPTITRACK_CHECKSUM_INIT_VAL 19

typedef struct {
	char c;
} optitrack_buf_c_t;

QueueHandle_t optitrack_queue;

optitrack_t optitrack;

void optitrack_init(int id)
{
	optitrack.id = id;
	optitrack_queue = xQueueCreate(OPTITRACK_QUEUE_SIZE, sizeof(optitrack_buf_c_t));
}

bool optitrack_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - optitrack.time_now) > 300) {
		return false;
	}
	return true;
}

void optitrack_buf_push(uint8_t c)
{
	if(optitrack.buf_pos >= OPTITRACK_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < OPTITRACK_SERIAL_MSG_SIZE; i++) {
			optitrack.buf[i - 1] = optitrack.buf[i];
		}

		/* save new byte to the last array element */
		optitrack.buf[OPTITRACK_SERIAL_MSG_SIZE - 1] = c;
		optitrack.buf_pos = OPTITRACK_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		optitrack.buf[optitrack.buf_pos] = c;
		optitrack.buf_pos++;
	}
}

void optitrack_isr_handler(uint8_t c)
{
	optitrack_buf_c_t optitrack_queue_item;
	optitrack_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(optitrack_queue, &optitrack_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void optitrack_update(void)
{
	optitrack_buf_c_t recept_c;
	while(xQueueReceive(optitrack_queue, &recept_c, 0) == pdTRUE) {
		uint8_t c = recept_c.c;

		optitrack_buf_push(c);
		if(c == '+' && optitrack.buf[0] == '@') {
			/* decode optitrack message */
			if(optitrack_serial_decoder(optitrack.buf) == 0) {
				optitrack.buf_pos = 0; //reset position pointer
			}
		}
	}
}

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
	const float dt = 1.0f / 120.0f; //fixed dt (120Hz)
	optitrack.vel_enu[0] = (optitrack.pos_enu[0] - optitrack.pos_last_enu[0]) / dt;
	optitrack.vel_enu[1] = (optitrack.pos_enu[1] - optitrack.pos_last_enu[1]) / dt;
	optitrack.vel_enu[2] = (optitrack.pos_enu[2] - optitrack.pos_last_enu[2]) / dt;

	float received_period = (optitrack.time_now - optitrack.time_last) * 0.001;
	optitrack.update_rate = 1.0f / received_period;

	optitrack.vel_filtered[0] = optitrack.vel_enu[0];
	optitrack.vel_filtered[1] = optitrack.vel_enu[1];
	optitrack.vel_filtered[2] = optitrack.vel_enu[2];
	//lpf(optitrack.vel_enu[0], &(optitrack.vel_filtered[0]), 0.8);
	//lpf(optitrack.vel_enu[1], &(optitrack.vel_filtered[1]), 0.8);
	//lpf(optitrack.vel_enu[2], &(optitrack.vel_filtered[2]), 0.8);
}

int optitrack_serial_decoder(uint8_t *buf)
{
	/* checksum validation */
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_optitrack_checksum_byte(&buf[3], OPTITRACK_SERIAL_MSG_SIZE - 4);
	int recv_id = buf[2];
	if(checksum != recv_checksum || optitrack.id != recv_id) {
		return 1; //error detected
	}

	/* update reception time */
	optitrack.time_now = get_sys_time_ms();

	/* decode position */
	memcpy(&optitrack.pos_enu[0], &buf[3], sizeof(float));
	memcpy(&optitrack.pos_enu[1], &buf[7], sizeof(float));
	memcpy(&optitrack.pos_enu[2], &buf[11], sizeof(float));

	/* decode quaternion, also swap the order to make the frame convention consist to the ahrs */
	memcpy(&optitrack.q[1], &buf[15], sizeof(float));
	memcpy(&optitrack.q[2], &buf[19], sizeof(float));
	memcpy(&optitrack.q[3], &buf[23], sizeof(float));
	memcpy(&optitrack.q[0], &buf[27], sizeof(float));
	optitrack.q[3] *= -1;

	/* first reception */
	if(optitrack.vel_ready == false) {
		optitrack.time_last = get_sys_time_ms();
		optitrack.pos_last_enu[0] = optitrack.pos_enu[0];
		optitrack.pos_last_enu[1] = optitrack.pos_enu[1];
		optitrack.pos_last_enu[2] = optitrack.pos_enu[2];
		optitrack.vel_enu[0] = 0.0f;
		optitrack.vel_enu[1] = 0.0f;
		optitrack.vel_enu[2] = 0.0f;
		optitrack.vel_ready = true;
		return 0;
	}

	/* calculate velocity with numerical differentiation */
	optitrack_numerical_vel_calc();
	optitrack.pos_last_enu[0] = optitrack.pos_enu[0]; //save for next iteration
	optitrack.pos_last_enu[1] = optitrack.pos_enu[1];
	optitrack.pos_last_enu[2] = optitrack.pos_enu[2];
	optitrack.time_last = optitrack.time_now;

	return 0;
}

void optitrack_get_position_enu(float *pos)
{
	pos[0] = optitrack.pos_enu[0];
	pos[1] = optitrack.pos_enu[1];
	pos[2] = optitrack.pos_enu[2];
}

float optitrack_get_position_enu_x(void)
{
	return optitrack.pos_enu[0];
}

float optitrack_get_position_enu_y(void)
{
	return optitrack.pos_enu[1];
}

float optitrack_get_position_enu_z(void)
{
	return optitrack.pos_enu[2];
}

void optitrack_get_velocity_enu(float *vel)
{
	vel[0] = optitrack.vel_enu[0];
	vel[1] = optitrack.vel_enu[1];
	vel[2] = optitrack.vel_enu[2];
}

float optitrack_get_velocity_enu_x(void)
{
	return optitrack.vel_enu[0];
}

float optitrack_get_velocity_enu_y(void)
{
	return optitrack.vel_enu[1];
}

float optitrack_get_velocity_enu_z(void)
{
	return optitrack.vel_enu[2];
}

void optitrack_get_position_ned(float *pos)
{
	pos[0] =  optitrack.pos_enu[1];
	pos[1] =  optitrack.pos_enu[0];
	pos[2] = -optitrack.pos_enu[2];
}

float optitrack_get_position_ned_x(void)
{
	return optitrack.pos_enu[1];
}

float optitrack_get_position_ned_y(void)
{
	return optitrack.pos_enu[0];
}

float optitrack_get_position_ned_z(void)
{
	return -optitrack.pos_enu[2];
}

void optitrack_get_velocity_ned(float *vel)
{
	vel[0] =  optitrack.vel_enu[1];
	vel[1] =  optitrack.vel_enu[0];
	vel[2] = -optitrack.vel_enu[2];
}

float optitrack_get_velocity_ned_x(void)
{
	return optitrack.vel_enu[1];
}

float optitrack_get_velocity_ned_y(void)
{
	return optitrack.vel_enu[0];
}

float optitrack_get_velocity_ned_z(void)
{
	return -optitrack.vel_enu[2];
}

void optitrack_get_quaternion(float *q)
{
	q[0] = optitrack.q[0];
	q[1] = optitrack.q[1];
	q[2] = optitrack.q[2];
	q[3] = optitrack.q[3];
}

void send_optitrack_position_debug_message(debug_msg_t *payload)
{
	//unit: [cm/s]
	float px = optitrack.pos_enu[0] * 100.0f;
	float py = optitrack.pos_enu[1] * 100.0f;
	float pz = optitrack.pos_enu[2] * 100.0f;

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_POSITION);
	pack_debug_debug_message_float(&px, payload);
	pack_debug_debug_message_float(&py, payload);
	pack_debug_debug_message_float(&pz, payload);
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
	//unit: [cm/s]
	float vx_raw = optitrack.vel_enu[0] * 100.0f;
	float vy_raw = optitrack.vel_enu[1] * 100.0f;
	float vz_raw = optitrack.vel_enu[2] * 100.0f;
	float vx_filtered = optitrack.vel_filtered[0] * 100.0f;
	float vy_filtered = optitrack.vel_filtered[1] * 100.0f;
	float vz_filtered = optitrack.vel_filtered[2] * 100.0f;

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_VELOCITY);
	pack_debug_debug_message_float(&vx_raw, payload);
	pack_debug_debug_message_float(&vy_raw, payload);
	pack_debug_debug_message_float(&vz_raw, payload);
	pack_debug_debug_message_float(&vx_filtered, payload);
	pack_debug_debug_message_float(&vy_filtered, payload);
	pack_debug_debug_message_float(&vz_filtered, payload);
	pack_debug_debug_message_float(&optitrack.update_rate, payload);
}
