#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "debug_link.h"
#include "delay.h"
#include "vector.h"
#include "matrix.h"
#include "imu.h"
#include "sbus_receiver.h"
#include "ahrs.h"
#include "multirotor_pid_ctrl.h"
#include "motor_thrust.h"
#include "led.h"
#include "optitrack.h"
#include "multirotor_geometry_ctrl.h"

extern imu_t imu;
extern ahrs_t ahrs;

extern optitrack_t optitrack;

extern pid_control_t pid_roll;
extern pid_control_t pid_pitch;
extern pid_control_t pid_yaw_rate;
extern pid_control_t pid_yaw;
extern pid_control_t pid_alt;
extern pid_control_t pid_alt_vel;
extern pid_control_t pid_vel_x;
extern pid_control_t pid_vel_y;
extern pid_control_t pid_pos_x;
extern pid_control_t pid_pos_y;
extern float motor1, motor2, motor3, motor4;

extern float nav_ctl_roll_command;
extern float nav_ctl_pitch_command;

extern float _mat_(P)[4 * 4];
extern float _mat_(K)[4 * 4];
extern float _mat_(eR)[3 * 1];
extern float _mat_(eW)[3 * 1];
extern float _mat_(J)[3 * 3];
radio_t rc;

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

static uint8_t generate_debug_debug_message_checksum(uint8_t *payload, int payload_count)
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

	checksum = generate_debug_debug_message_checksum(payload + 3, payload_count - 3);

	payload[0] = '@';
	payload[1] = payload_count - 3;

	payload[payload_count] = checksum;
	payload_count++;

	uart3_puts((char *)payload, payload_count);
}

void send_imu_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_IMU);
	pack_debug_debug_message_float(&imu.accel_raw.x, payload);
	pack_debug_debug_message_float(&imu.accel_raw.y, payload);
	pack_debug_debug_message_float(&imu.accel_raw.z, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.x, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.y, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.z, payload);
	pack_debug_debug_message_float(&imu.gyro_raw.x, payload);
	pack_debug_debug_message_float(&imu.gyro_raw.y, payload);
	pack_debug_debug_message_float(&imu.gyro_raw.z, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.x, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.y, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.z, payload);
}

void send_attitude_euler_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_EULER);
	pack_debug_debug_message_float(&ahrs.attitude.roll, payload);
	pack_debug_debug_message_float(&ahrs.attitude.pitch, payload);
	pack_debug_debug_message_float(&ahrs.attitude.yaw, payload);
}

void send_attitude_quaternion_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_QUAT);
	pack_debug_debug_message_float(&ahrs.q[0], payload);
	pack_debug_debug_message_float(&ahrs.q[1], payload);
	pack_debug_debug_message_float(&ahrs.q[2], payload);
	pack_debug_debug_message_float(&ahrs.q[3], payload);
}

void send_attitude_imu_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_IMU);
	pack_debug_debug_message_float(&ahrs.attitude.roll, payload);
	pack_debug_debug_message_float(&ahrs.attitude.pitch, payload);
	pack_debug_debug_message_float(&ahrs.attitude.yaw, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.x, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.y, payload);
	pack_debug_debug_message_float(&imu.accel_lpf.z, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.x, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.y, payload);
	pack_debug_debug_message_float(&imu.gyro_lpf.z, payload);
}

void send_ekf_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_EKF);
	pack_debug_debug_message_float(&_mat_(P)[0], payload);
	pack_debug_debug_message_float(&_mat_(P)[5], payload);
	pack_debug_debug_message_float(&_mat_(P)[10], payload);
	pack_debug_debug_message_float(&_mat_(P)[15], payload);
	pack_debug_debug_message_float(&_mat_(K)[0], payload);
	pack_debug_debug_message_float(&_mat_(K)[5], payload);
	pack_debug_debug_message_float(&_mat_(K)[10], payload);
	pack_debug_debug_message_float(&_mat_(K)[15], payload);
}

void send_motor_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_MOTOR);
	pack_debug_debug_message_float(&motor1, payload);
	pack_debug_debug_message_float(&motor2, payload);
	pack_debug_debug_message_float(&motor3, payload);
	pack_debug_debug_message_float(&motor4, payload);
}

void send_pid_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_PID_DEBUG);
#if 1
	//roll pd control
	pack_debug_debug_message_float(&pid_roll.error_current, payload);
	pack_debug_debug_message_float(&pid_roll.error_derivative, payload);
	pack_debug_debug_message_float(&pid_roll.p_final, payload);
	pack_debug_debug_message_float(&pid_roll.i_final, payload);
	pack_debug_debug_message_float(&pid_roll.d_final, payload);
	pack_debug_debug_message_float(&pid_roll.output, payload);
#endif

#if 0
	pack_debug_debug_message_float(&pid_pitch.error_current, payload);
	pack_debug_debug_message_float(&pid_pitch.error_derivative, payload);
	pack_debug_debug_message_float(&pid_pitch.p_final, payload);
	pack_debug_debug_message_float(&pid_pitch.i_final, payload);
	pack_debug_debug_message_float(&pid_pitch.d_final, payload);
	pack_debug_debug_message_float(&pid_pitch.output, payload);
#endif

#if 0
	pack_debug_debug_message_float(&pid_yaw_rate.error_current, payload);
	pack_debug_debug_message_float(&pid_yaw_rate.error_derivative, payload);
	pack_debug_debug_message_float(&pid_yaw_rate.p_final, payload);
	pack_debug_debug_message_float(&pid_yaw_rate.i_final, payload);
	pack_debug_debug_message_float(&pid_yaw_rate.d_final, payload);
	pack_debug_debug_message_float(&pid_yaw_rate.output, payload);
#endif
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

void send_general_float_debug_message(float val, debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_GENERAL_FLOAT);
	pack_debug_debug_message_float(&val, payload);
}

void send_accel_calib_debug_message(void)
{
	char s[100] = {0.0};
	sprintf(s, "x:%d, y:%d, z:%d\n\r", imu.accel_unscaled.x, imu.accel_unscaled.y, imu.accel_unscaled.z);
	uart3_puts(s, strlen(s));
}

void send_accel_bias_calib_debug_message(void)
{
	char s[100] = {0.0};
	int bias_x = imu.accel_unscaled.x;
	int bias_y = imu.accel_unscaled.y;
	int bias_z = imu.accel_unscaled.z - 2048; //2048 for +-16g configuration
	sprintf(s, "x:%d, y:%d, z:%d\n\r", bias_x, bias_y, bias_z);
	uart3_puts(s, strlen(s));
}

void task_debug_link(void *param)
{
	debug_msg_t payload;

	float update_rate = 50;
	float delay_time_ms = (1.0f / update_rate) * 1000.0f;

	while(1) {
		//send_imu_debug_message(&payload);
		//send_attitude_euler_debug_message(&payload);
		send_attitude_quaternion_debug_message(&payload);
		//send_attitude_imu_debug_message(&payload);
		//send_ekf_debug_message(&payload);
		//send_pid_debug_message(&payload);
		//send_motor_debug_message(&payload);
		//send_optitrack_position_debug_message(&payload);
		//send_optitrack_quaternion_debug_message(&payload);
		//send_optitrack_velocity_debug_message(&payload);
		//send_general_float_debug_message(optitrack.recv_freq, &payload);
		//send_general_float_debug_message(pid_pos_x.error_current, &payload);
		//send_general_float_debug_message(motor_cmd[0], &payload);
		//send_accel_calib_debug_message();
		//send_accel_bias_calib_debug_message();
		//send_geometry_ctrl_debug(&payload);
		//send_uav_dynamics_debug(&payload);
		send_onboard_data(payload.s, payload.len);
		freertos_task_delay(delay_time_ms);
	}
}
