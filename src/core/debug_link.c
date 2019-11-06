#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "uart.h"
#include "debug_link.h"
#include "delay.h"
#include "vector.h"
#include "matrix.h"
#include "ahrs.h"
#include "flight_ctl.h"

extern imu_t imu;
extern ahrs_t ahrs;

extern pid_control_t pid_roll;
extern pid_control_t pid_pitch;
extern pid_control_t pid_yaw_rate;

extern float motor1, motor2, motor3, motor4;

extern float _mat_(P)[4 * 4];
extern float _mat_(K)[4 * 4];

int pack_float(float *data_float, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_float, sizeof(float));
	return sizeof(float);
}

int pack_vector3d(vector3d_f_t *data_vector3d, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_vector3d, sizeof(vector3d_f_t));
	return sizeof(vector3d_f_t);
}

int pack_attitude(euler_t *data_attitude, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_attitude, sizeof(euler_t));
	return sizeof(euler_t);
}

static uint8_t generate_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

static void send_onboard_data(uint8_t *payload, int payload_count)
{
	uint8_t checksum;

	checksum = generate_checksum_byte(payload + 3, payload_count - 3);

	payload[0] = '@';
	payload[1] = payload_count - 3;

	payload[payload_count] = checksum;
	payload_count++;

	uart3_puts((char *)payload, payload_count);
}

void send_imu_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_IMU;

	payload->len += pack_vector3d(&imu.raw_accel, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.filtered_accel, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.raw_gyro, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.filtered_gyro, payload->s + payload->len);
}

void send_attitude_euler_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_ATTITUDE_EULER;

	payload->len += pack_attitude(&ahrs.attitude, payload->s + payload->len);
}

void send_attitude_quaternion_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_ATTITUDE_QUAT;

	payload->len += pack_float(&ahrs.q[0], payload->s + payload->len);
	payload->len += pack_float(&ahrs.q[1], payload->s + payload->len);
	payload->len += pack_float(&ahrs.q[2], payload->s + payload->len);
	payload->len += pack_float(&ahrs.q[3], payload->s + payload->len);
}

void send_attitude_imu_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_ATTITUDE_IMU;

	payload->len += pack_attitude(&ahrs.attitude, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.filtered_accel, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.filtered_gyro, payload->s + payload->len);
}

void send_ekf_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_EKF;

	payload->len += pack_float(&_mat_(P)[0], payload->s + payload->len);
	payload->len += pack_float(&_mat_(P)[5], payload->s + payload->len);
	payload->len += pack_float(&_mat_(P)[10], payload->s + payload->len);
	payload->len += pack_float(&_mat_(P)[15], payload->s + payload->len);
	payload->len += pack_float(&_mat_(K)[0], payload->s + payload->len);
	payload->len += pack_float(&_mat_(K)[5], payload->s + payload->len);
	payload->len += pack_float(&_mat_(K)[10], payload->s + payload->len);
	payload->len += pack_float(&_mat_(K)[15], payload->s + payload->len);
}

void send_motor_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_MOTOR;

	payload->len += pack_float(&motor1, payload->s + payload->len);
	payload->len += pack_float(&motor2, payload->s + payload->len);
	payload->len += pack_float(&motor3, payload->s + payload->len);
	payload->len += pack_float(&motor4, payload->s + payload->len);
}

void send_pid_debug(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_PID_DEBUG;
#if 1
	//roll pd control
	payload->len += pack_float(&pid_roll.error_current, payload->s + payload->len);
	payload->len += pack_float(&pid_roll.error_derivative, payload->s + payload->len);
	payload->len += pack_float(&pid_roll.p_final, payload->s + payload->len);
	payload->len += pack_float(&pid_roll.i_final, payload->s + payload->len);
	payload->len += pack_float(&pid_roll.d_final, payload->s + payload->len);
	payload->len += pack_float(&pid_roll.output, payload->s + payload->len);
#endif

#if 0
	payload->len += pack_float(&pid_pitch.error_current, payload->s + payload->len);
	payload->len += pack_float(&pid_pitch.error_derivative, payload->s + payload->len);
	payload->len += pack_float(&pid_pitch.p_final, payload->s + payload->len);
	payload->len += pack_float(&pid_pitch.i_final, payload->s + payload->len);
	payload->len += pack_float(&pid_pitch.d_final, payload->s + payload->len);
	payload->len += pack_float(&pid_pitch.output, payload->s + payload->len);
#endif

#if 0
	payload->len += pack_float(&pid_yaw_rate.error_current, payload->s + payload->len);
	payload->len += pack_float(&pid_yaw_rate.error_derivative, payload->s + payload->len);
	payload->len += pack_float(&pid_yaw_rate.p_final, payload->s + payload->len);
	payload->len += pack_float(&pid_yaw_rate.i_final, payload->s + payload->len);
	payload->len += pack_float(&pid_yaw_rate.d_final, payload->s + payload->len);
	payload->len += pack_float(&pid_yaw_rate.output, payload->s + payload->len);
#endif
}

void task_debug_link(void *param)
{
	debug_msg_t payload;

	while(1) {
		send_imu_message(&payload);
		//send_attitude_euler_message(&payload);
		//send_attitude_quaternion_message(&payload);
		//send_attitude_imu_message(&payload);
		//send_ekf_message(&payload);
		//send_pid_debug(&payload);
		//send_motor_message(&payload);
		send_onboard_data(payload.s, payload.len);
		vTaskDelay(1000);
	}
}
