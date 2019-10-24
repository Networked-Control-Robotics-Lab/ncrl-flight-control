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

int pack_attitude(attitude_t *data_attitude, uint8_t *byte_to_sent)
{
	memcpy(byte_to_sent, (uint8_t *)data_attitude, sizeof(attitude_t));
	return sizeof(attitude_t);
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

void send_imu_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_IMU;

	payload_size += pack_vector3d(&imu.raw_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.raw_gyro, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_gyro, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_attitude_euler_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message
	payload[2] = MESSAGE_ID_ATTITUDE_EULER;

	payload_size += pack_attitude(&ahrs.attitude, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_attitude_quaternion_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_ATTITUDE_QUAT;

	payload_size += pack_float(&ahrs.attitude.q[0], payload + payload_size);
	payload_size += pack_float(&ahrs.attitude.q[1], payload + payload_size);
	payload_size += pack_float(&ahrs.attitude.q[2], payload + payload_size);
	payload_size += pack_float(&ahrs.attitude.q[3], payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_attitude_imu_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_ATTITUDE_IMU;

	payload_size += pack_attitude(&ahrs.attitude, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_accel, payload + payload_size);
	payload_size += pack_vector3d(&imu.filtered_gyro, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_ekf_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_EKF;

	payload_size += pack_float(&_mat_(P)[0], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[5], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[10], payload + payload_size);
	payload_size += pack_float(&_mat_(P)[15], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[0], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[5], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[10], payload + payload_size);
	payload_size += pack_float(&_mat_(K)[15], payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_motor_message(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_MOTOR;

	payload_size += pack_float(&motor1, payload + payload_size);
	payload_size += pack_float(&motor2, payload + payload_size);
	payload_size += pack_float(&motor3, payload + payload_size);
	payload_size += pack_float(&motor4, payload + payload_size);

	send_onboard_data(payload, payload_size);
}

void send_pid_debug(void)
{
	uint8_t payload[512] = {0}; //~64 float
	int payload_size = 3; //reserved for header message

	payload[2] = MESSAGE_ID_PID_DEBUG;

#if 0
	//roll pd control
	payload_size += pack_float(&pid_roll.error_current, payload + payload_size);
	payload_size += pack_float(&pid_roll.error_derivative, payload + payload_size);
	payload_size += pack_float(&pid_roll.p_final, payload + payload_size);
	payload_size += pack_float(&pid_roll.i_final, payload + payload_size);
	payload_size += pack_float(&pid_roll.d_final, payload + payload_size);
	payload_size += pack_float(&pid_roll.output, payload + payload_size);
#endif

#if 1
	payload_size += pack_float(&pid_pitch.error_current, payload + payload_size);
	payload_size += pack_float(&pid_pitch.error_derivative, payload + payload_size);
	payload_size += pack_float(&pid_pitch.p_final, payload + payload_size);
	payload_size += pack_float(&pid_pitch.i_final, payload + payload_size);
	payload_size += pack_float(&pid_pitch.d_final, payload + payload_size);
	payload_size += pack_float(&pid_pitch.output, payload + payload_size);
#endif

#if 0
	payload_size += pack_float(&pid_yaw_rate.error_current, payload + payload_size);
	payload_size += pack_float(&pid_yaw_rate.error_derivative, payload + payload_size);
	payload_size += pack_float(&pid_yaw_rate.p_final, payload + payload_size);
	payload_size += pack_float(&pid_yaw_rate.i_final, payload + payload_size);
	payload_size += pack_float(&pid_yaw_rate.d_final, payload + payload_size);
	payload_size += pack_float(&pid_yaw_rate.output, payload + payload_size);
#endif


	send_onboard_data(payload, payload_size);
}

void task_debug_link(void *param)
{
	while(1) {
		blocked_delay_ms(10);

		//send_imu_message();
		//send_attitude_euler_message();
		send_attitude_quaternion_message();
		//send_attitude_imu_message();
		//send_ekf_message();
		//send_pid_debug();
		//send_motor_message();
		taskYIELD();
	}
}
