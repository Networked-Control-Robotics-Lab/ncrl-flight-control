#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx_conf.h"
#include "uart.h"
#include "imu.h"
#include "gpio.h"
#include "vins_mono.h"
#include "sys_time.h"

#define VINS_MONO_IMU_MSG_SIZE 27
#define VINS_MONO_CHECKSUM_INIT_VAL 19
#define VINS_MONO_QUEUE_SIZE (32 * 400) //~400 packets

typedef struct {
	char c;
} vins_mono_buf_c_t;

QueueHandle_t vins_mono_queue;

vins_mono_t vins_mono;

void vins_mono_init(int id)
{
	vins_mono.id = id;
	vins_mono_queue = xQueueCreate(VINS_MONO_QUEUE_SIZE, sizeof(vins_mono_buf_c_t));
}

bool vins_mono_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - vins_mono.time_now) > 300) {
		led_off(LED_G);
		return false;
	}
	return true;
}

void vins_mono_numerical_vel_calc(void)
{
	const float dt = 1.0f / 120.0f; //fixed dt (120Hz)
	vins_mono.vel_raw[0] = (vins_mono.pos[0] - vins_mono.pos_last[0]) / dt;
	vins_mono.vel_raw[1] = (vins_mono.pos[1] - vins_mono.pos_last[1]) / dt;
	vins_mono.vel_raw[2] = (vins_mono.pos[2] - vins_mono.pos_last[2]) / dt;

	float received_period = (vins_mono.time_now - vins_mono.time_last) * 0.001;
	vins_mono.update_rate = 1.0f / received_period;

	vins_mono.vel_filtered[0] = vins_mono.vel_raw[0];
	vins_mono.vel_filtered[1] = vins_mono.vel_raw[1];
	vins_mono.vel_filtered[2] = vins_mono.vel_raw[2];
	//lpf(vins_mono.vel_raw[0], &(vins_mono.vel_filtered[0]), 0.8);
	//lpf(vins_mono.vel_raw[1], &(vins_mono.vel_filtered[1]), 0.8);
	//lpf(vins_mono.vel_raw[2], &(vins_mono.vel_filtered[2]), 0.8);
}

#define VINS_MONO_CHECKSUM_INIT_VAL 19
static uint8_t generate_vins_mono_checksum_byte(uint8_t *payload, int payload_cnt)
{
	uint8_t result = VINS_MONO_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_cnt; i++)
		result ^= payload[i];

	return result;
}

void vins_mono_buf_push(uint8_t c)
{
	if(vins_mono.buf_pos >= VINS_MONO_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < VINS_MONO_SERIAL_MSG_SIZE; i++) {
			vins_mono.buf[i - 1] = vins_mono.buf[i];
		}

		/* save new byte to the last array element */
		vins_mono.buf[VINS_MONO_SERIAL_MSG_SIZE - 1] = c;
		vins_mono.buf_pos = VINS_MONO_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		vins_mono.buf[vins_mono.buf_pos] = c;
		vins_mono.buf_pos++;
	}
}

void vins_mono_isr_handler(uint8_t c)
{
	vins_mono_buf_c_t vins_mono_queue_item;
	vins_mono_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(vins_mono_queue, &vins_mono_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

void vins_mono_update(void)
{
	vins_mono_buf_c_t recept_c;
	while(xQueueReceive(vins_mono_queue, &recept_c, 0) == pdTRUE) {
		uint8_t c = recept_c.c;

		vins_mono_buf_push(c);
		if(c == '+' && vins_mono.buf[0] == '@') {
			/* decode optitrack message */
			if(vins_mono_serial_decoder(vins_mono.buf) == 0) {
				led_on(LED_G);
				vins_mono.buf_pos = 0; //reset position pointer
			}
		}
	}
}

int vins_mono_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_vins_mono_checksum_byte(&buf[3], VINS_MONO_SERIAL_MSG_SIZE - 4);
	int recv_id = buf[2];
	if(checksum != recv_checksum || vins_mono.id != recv_id) {
		return 1; //error detected
	}

	vins_mono.time_now = get_sys_time_ms();

	float enu_pos_x, enu_pos_y, enu_pos_z;

	memcpy(&enu_pos_x, &buf[3], sizeof(float)); //in ned coordinate system
	memcpy(&enu_pos_y, &buf[7], sizeof(float));
	memcpy(&enu_pos_z, &buf[11], sizeof(float));
	vins_mono.pos[0] = enu_pos_x; //east
	vins_mono.pos[1] = enu_pos_y; //north
	vins_mono.pos[2] = enu_pos_z; //up
	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&vins_mono.q[1], &buf[15], sizeof(float));
	memcpy(&vins_mono.q[2], &buf[19], sizeof(float));
	memcpy(&vins_mono.q[3], &buf[23], sizeof(float));
	memcpy(&vins_mono.q[0], &buf[27], sizeof(float));
	vins_mono.q[3] *= -1;

	if(vins_mono.vel_ready == false) {
		vins_mono.time_last = get_sys_time_ms();
		vins_mono.pos_last[0] = vins_mono.pos[0];
		vins_mono.pos_last[1] = vins_mono.pos[1];
		vins_mono.pos_last[2] = vins_mono.pos[2];
		vins_mono.vel_raw[0] = 0.0f;
		vins_mono.vel_raw[1] = 0.0f;
		vins_mono.vel_raw[2] = 0.0f;
		vins_mono.vel_ready = true;
		return 0;
	}

	vins_mono_numerical_vel_calc();
	vins_mono.pos_last[0] = vins_mono.pos[0]; //save for next iteration
	vins_mono.pos_last[1] = vins_mono.pos[1];
	vins_mono.pos_last[2] = vins_mono.pos[2];
	vins_mono.time_last = vins_mono.time_now;

	return 0;
}

void vins_mono_read_pos_x(float *x)
{
	*x = vins_mono.pos[0];
}

void vins_mono_read_pos_y(float *y)
{
	*y = vins_mono.pos[1];
}

void vins_mono_read_pos_z(float *z)
{
	*z = vins_mono.pos[2];
}

void vins_mono_read_vel_x(float *vx)
{
	*vx = vins_mono.vel_raw[0];
}

void vins_mono_read_vel_y(float *vy)
{
	*vy = vins_mono.vel_raw[1];
}

void vins_mono_read_vel_z(float *vz)
{
	*vz = vins_mono.vel_raw[2];
}


void send_vins_mono_imu_msg(void)
{
	/*+------------+----------+---------+---------+---------+--------+--------+--------+----------+
	 *| start byte | checksum | accel_x | accel_y | accel_z | gyro_x | gyro_y | gyro_z | end byte |
	 *+------------+----------+---------+---------+---------+--------+--------+--------+----------+*/

	float accel[3] = {0.0f};
	float gyro[3] = {0.0f};

	get_accel_lpf(accel);
	get_gyro_lpf(gyro);

	char msg_buf[VINS_MONO_IMU_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0;
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	memcpy(msg_buf + msg_pos, &accel[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &accel[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &accel[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &gyro[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &gyro[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &gyro[2], sizeof(float));
	msg_pos += sizeof(float);

	msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	msg_buf[1] = generate_vins_mono_checksum_byte((uint8_t *)&msg_buf[3],
	                VINS_MONO_IMU_MSG_SIZE - 3);

	uart6_puts(msg_buf, VINS_MONO_IMU_MSG_SIZE);
}

void vins_mono_send_imu_50hz(void)
{
	/* triggered every 8 times since the function is designed to be called by
	 * flight control main loop (400Hz) */
	static int prescaler = 2;
	prescaler--;

	if(prescaler == 0) {
		send_vins_mono_imu_msg();
		prescaler = 2;
	}
}

void vins_mono_camera_trigger_20hz(void)
{
	/* to generate the camera trigger pulse:
	 * (1/20Hz) / (1/400Hz) = 20 (flight control loop is 20x faster than what we need)
	 * 10% on:  20 * 0.1 = 2times
	 * 90% off: 20 * 0.9 = 18times*/

	static int counter = 0;

	if(counter < 2) {
		gpio_on(MOTOR8);
	} else {
		gpio_off(MOTOR8);
	}

	counter = (counter + 1) % 20;
}

void send_vins_mono_position_debug_message(debug_msg_t *payload)
{
	float px = vins_mono.pos[0] * 100.0f; //[cm]
	float py = vins_mono.pos[1] * 100.0f; //[cm]
	float pz = vins_mono.pos[2] * 100.0f; //[cm]

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_POSITION);
	pack_debug_debug_message_float(&px, payload);
	pack_debug_debug_message_float(&py, payload);
	pack_debug_debug_message_float(&pz, payload);
}

void send_vins_mono_quaternion_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_QUATERNION);
	pack_debug_debug_message_float(&vins_mono.q[0], payload);
	pack_debug_debug_message_float(&vins_mono.q[1], payload);
	pack_debug_debug_message_float(&vins_mono.q[2], payload);
	pack_debug_debug_message_float(&vins_mono.q[3], payload);
}

void send_vins_mono_velocity_debug_message(debug_msg_t *payload)
{
	float vx_raw = vins_mono.vel_raw[0] * 100.0f; //[cm/s]
	float vy_raw = vins_mono.vel_raw[1] * 100.0f; //[cm/s]
	float vz_raw = vins_mono.vel_raw[2] * 100.0f; //[cm/s]
	float vx_filtered = vins_mono.vel_filtered[0] * 100.0f; //[cm/s]
	float vy_filtered = vins_mono.vel_filtered[1] * 100.0f; //[cm/s]
	float vz_filtered = vins_mono.vel_filtered[2] * 100.0f; //[cm/s]

	pack_debug_debug_message_header(payload, MESSAGE_ID_OPTITRACK_VELOCITY);
	pack_debug_debug_message_float(&vx_raw, payload);
	pack_debug_debug_message_float(&vy_raw, payload);
	pack_debug_debug_message_float(&vz_raw, payload);
	pack_debug_debug_message_float(&vx_filtered, payload);
	pack_debug_debug_message_float(&vy_filtered, payload);
	pack_debug_debug_message_float(&vz_filtered, payload);
	pack_debug_debug_message_float(&vins_mono.update_rate, payload);
}
