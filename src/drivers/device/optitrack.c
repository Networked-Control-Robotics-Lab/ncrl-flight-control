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

#define OPTITRACK_QUEUE_SIZE (32 * 400) //~400 packets

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
	const float dt = 1.0f / 120.0f; //fixed dt (120Hz)
	optitrack.vel_raw[0] = (optitrack.pos[0] - optitrack.pos_last[0]) / dt;
	optitrack.vel_raw[1] = (optitrack.pos[1] - optitrack.pos_last[1]) / dt;
	optitrack.vel_raw[2] = (optitrack.pos[2] - optitrack.pos_last[2]) / dt;

	float received_period = (optitrack.time_now - optitrack.time_last) * 0.001;
	optitrack.update_rate = 1.0f / received_period;

	optitrack.vel_filtered[0] = optitrack.vel_raw[0];
	optitrack.vel_filtered[1] = optitrack.vel_raw[1];
	optitrack.vel_filtered[2] = optitrack.vel_raw[2];
	//lpf(optitrack.vel_raw[0], &(optitrack.vel_filtered[0]), 0.8);
	//lpf(optitrack.vel_raw[1], &(optitrack.vel_filtered[1]), 0.8);
	//lpf(optitrack.vel_raw[2], &(optitrack.vel_filtered[2]), 0.8);
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

	float enu_pos_x, enu_pos_y, enu_pos_z;

	memcpy(&enu_pos_x, &buf[3], sizeof(float)); //in ned coordinate system
	memcpy(&enu_pos_y, &buf[7], sizeof(float));
	memcpy(&enu_pos_z, &buf[11], sizeof(float));
	optitrack.pos[0] = enu_pos_x; //east
	optitrack.pos[1] = enu_pos_y; //north
	optitrack.pos[2] = enu_pos_z; //up
	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&optitrack.q[1], &buf[15], sizeof(float));
	memcpy(&optitrack.q[2], &buf[19], sizeof(float));
	memcpy(&optitrack.q[3], &buf[23], sizeof(float));
	memcpy(&optitrack.q[0], &buf[27], sizeof(float));
	optitrack.q[3] *= -1;

	if(optitrack.vel_ready == false) {
		optitrack.time_last = get_sys_time_ms();
		optitrack.pos_last[0] = optitrack.pos[0];
		optitrack.pos_last[1] = optitrack.pos[1];
		optitrack.pos_last[2] = optitrack.pos[2];
		optitrack.vel_raw[0] = 0.0f;
		optitrack.vel_raw[1] = 0.0f;
		optitrack.vel_raw[2] = 0.0f;
		optitrack.vel_ready = true;
		return 0;
	}

	optitrack_numerical_vel_calc();
	optitrack.pos_last[0] = optitrack.pos[0]; //save for next iteration
	optitrack.pos_last[1] = optitrack.pos[1];
	optitrack.pos_last[2] = optitrack.pos[2];
	optitrack.time_last = optitrack.time_now;

	return 0;
}

float optitrack_read_pos_x(void)
{
	return optitrack.pos[0];
}

float optitrack_read_pos_y(void)
{
	return optitrack.pos[1];
}

float optitrack_read_pos_z(void)
{
	return optitrack.pos[2];
}

float optitrack_read_vel_x(void)
{
	return optitrack.vel_raw[0];
}

float optitrack_read_vel_y(void)
{
	return optitrack.vel_raw[1];
}

float optitrack_read_vel_z(void)
{
	return optitrack.vel_raw[2];
}

void send_optitrack_position_debug_message(debug_msg_t *payload)
{
	float px = optitrack.pos[0] * 100.0f; //[cm]
	float py = optitrack.pos[1] * 100.0f; //[cm]
	float pz = optitrack.pos[2] * 100.0f; //[cm]

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
	float vx_raw = optitrack.vel_raw[0] * 100.0f; //[cm/s]
	float vy_raw = optitrack.vel_raw[1] * 100.0f; //[cm/s]
	float vz_raw = optitrack.vel_raw[2] * 100.0f; //[cm/s]
	float vx_filtered = optitrack.vel_filtered[0] * 100.0f; //[cm/s]
	float vy_filtered = optitrack.vel_filtered[1] * 100.0f; //[cm/s]
	float vz_filtered = optitrack.vel_filtered[2] * 100.0f; //[cm/s]

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_VELOCITY);
	pack_debug_debug_message_float(&vx_raw, payload);
	pack_debug_debug_message_float(&vy_raw, payload);
	pack_debug_debug_message_float(&vz_raw, payload);
	pack_debug_debug_message_float(&vx_filtered, payload);
	pack_debug_debug_message_float(&vy_filtered, payload);
	pack_debug_debug_message_float(&vz_filtered, payload);
	pack_debug_debug_message_float(&optitrack.update_rate, payload);
}
