#include <stdint.h>
#include <string.h>
#include "uart.h"
#include "imu.h"
#include "gpio.h"

#define VINS_MONO_IMU_MSG_SIZE 27
#define VINS_MONO_CHECKSUM_INIT_VAL 19

static uint8_t generate_vins_mono_checksum_byte(uint8_t *payload, int payload_cnt)
{
	uint8_t result = VINS_MONO_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_cnt; i++)
		result ^= payload[i];

	return result;
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
