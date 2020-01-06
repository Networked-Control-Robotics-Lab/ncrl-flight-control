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
#include "attitude_pd_ctrl.h"
#include "motor_thrust.h"
#include "led.h"
#include "optitrack.h"

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
extern float _mat_(inertia_effect)[3 * 1];
extern float geometry_ctrl_forces[4];
radio_t rc;

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

	payload->len += pack_vector3d(&imu.accel_raw, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.accel_lpf, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.gyro_raw, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.gyro_lpf, payload->s + payload->len);
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
	payload->len += pack_vector3d(&imu.accel_lpf, payload->s + payload->len);
	payload->len += pack_vector3d(&imu.gyro_lpf, payload->s + payload->len);
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

void send_geometry_ctrl_debug(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	float roll_error = rad_to_deg(_mat_(eR)[0]);
	float pitch_error = rad_to_deg(_mat_(eR)[1]);
	float yaw_error = rad_to_deg(_mat_(eR)[2]);

	float wx_error = rad_to_deg(_mat_(eW)[0]);
	float wy_error = rad_to_deg(_mat_(eW)[1]);
	float wz_error = rad_to_deg(_mat_(eW)[2]);

	payload->s[2] = MESSAGE_ID_GEOMETRY_DEBUG;
	payload->len += pack_float(&roll_error, payload->s + payload->len);
	payload->len += pack_float(&pitch_error, payload->s + payload->len);
	payload->len += pack_float(&yaw_error, payload->s + payload->len);
	payload->len += pack_float(&wx_error, payload->s + payload->len);
	payload->len += pack_float(&wy_error, payload->s + payload->len);
	payload->len += pack_float(&wz_error, payload->s + payload->len);
	payload->len += pack_float(&_mat_(inertia_effect)[0], payload->s + payload->len);
	payload->len += pack_float(&_mat_(inertia_effect)[1], payload->s + payload->len);
	payload->len += pack_float(&_mat_(inertia_effect)[2], payload->s + payload->len);
	payload->len += pack_float(&geometry_ctrl_forces[0], payload->s + payload->len);
	payload->len += pack_float(&geometry_ctrl_forces[1], payload->s + payload->len);
	payload->len += pack_float(&geometry_ctrl_forces[2], payload->s + payload->len);
	payload->len += pack_float(&geometry_ctrl_forces[3], payload->s + payload->len);
}

void send_optitrack_position_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_OPTITRACK_POSITION;

	payload->len += pack_float(&optitrack.pos_x, payload->s + payload->len);
	payload->len += pack_float(&optitrack.pos_y, payload->s + payload->len);
	payload->len += pack_float(&optitrack.pos_z, payload->s + payload->len);
}

void send_optitrack_quaternion_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_OPTITRACK_QUATERNION;

	payload->len += pack_float(&optitrack.q[0], payload->s + payload->len);
	payload->len += pack_float(&optitrack.q[1], payload->s + payload->len);
	payload->len += pack_float(&optitrack.q[2], payload->s + payload->len);
	payload->len += pack_float(&optitrack.q[3], payload->s + payload->len);
}

void send_optitrack_velocity_message(debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_OPTITRACK_VELOCITY;

	payload->len += pack_float(&optitrack.vel_raw_x, payload->s + payload->len);
	payload->len += pack_float(&optitrack.vel_raw_y, payload->s + payload->len);
	payload->len += pack_float(&optitrack.vel_raw_z, payload->s + payload->len);
	payload->len += pack_float(&optitrack.vel_lpf_x, payload->s + payload->len);
	payload->len += pack_float(&optitrack.vel_lpf_y, payload->s + payload->len);
	payload->len += pack_float(&optitrack.vel_lpf_z, payload->s + payload->len);
}

void send_general_float_message(float val, debug_msg_t *payload)
{
	payload->len = 3; //reserve for header message

	payload->s[2] = MESSAGE_ID_GENERAL_FLOAT;

	payload->len += pack_float(&val, payload->s + payload->len);
}

void send_accel_calib_message(void)
{
	char s[100] = {0.0};
	sprintf(s, "x:%d, y:%d, z:%d\n\r", imu.accel_unscaled.x, imu.accel_unscaled.y, imu.accel_unscaled.z);
	uart3_puts(s, strlen(s));
}

void send_accel_bias_calib_message(void)
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
		//send_imu_message(&payload);
		//send_attitude_euler_message(&payload);
		//send_attitude_quaternion_message(&payload);
		//send_attitude_imu_message(&payload);
		//send_ekf_message(&payload);
		//send_pid_debug(&payload);
		//send_motor_message(&payload);
		//send_optitrack_position_message(&payload);
		//send_optitrack_quaternion_message(&payload);
		//send_optitrack_velocity_message(&payload);
		//send_general_float_message(optitrack.recv_freq, &payload);
		//send_general_float_message(pid_pos_x.error_current, &payload);
		//send_general_float_message(motor_cmd[0], &payload);
		//send_accel_calib_message();
		//send_accel_bias_calib_message();
		send_geometry_ctrl_debug(&payload);
		send_onboard_data(payload.s, payload.len);
		freertos_task_delay(delay_time_ms);
	}
}
