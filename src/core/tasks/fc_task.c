#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "bound.h"
#include "led.h"
#include "uart.h"
#include "pwm.h"
#include "sbus_receiver.h"
#include "mpu6500.h"
#include "motor.h"
#include "optitrack.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "madgwick_ahrs.h"
#include "multirotor_pid_ctrl.h"
#include "multirotor_geometry_ctrl.h"
#include "motor_thrust.h"
#include "fc_task.h"
#include "sys_time.h"
#include "proj_config.h"

#define FLIGHT_CTL_PRESCALER_RELOAD 10

SemaphoreHandle_t flight_ctl_semphr;

imu_t imu;
ahrs_t ahrs;
radio_t rc;

float uav_dynamics_m[3] = {0.0f};
float uav_dynamics_m_rot_frame[3] = {0.0f};

float motor_cmd[4];

void flight_ctl_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(flight_ctl_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void rc_safety_protection(void)
{
	radio_t rc;

	float time_last = 0.0f;
	float time_current = 0.0f;

	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	do {
		time_current = get_sys_time_ms();
		if(time_current - time_last > 100.0f) {
			led_toggle(LED_R);
			time_last = time_current;
		}
		read_rc_info(&rc);
	} while(rc_safety_check(&rc) == 1);
}

void task_flight_ctl(void *param)
{
	euler_t att_euler_est;

	mpu6500_init(&imu);
	motor_init();

	ahrs_init(imu.accel_raw);
	madgwick_t madgwick_ahrs_info;
	madgwick_init(&madgwick_ahrs_info, 400, 0.4);

	multirotor_pid_controller_init();

	geometry_ctrl_init();

	led_off(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	rc_safety_protection();

	while(1) {
		while(xSemaphoreTake(flight_ctl_semphr, 9) == pdFALSE);

		//gpio_toggle(MOTOR7_FREQ_TEST);

		read_rc_info(&rc);

#if (SELECT_AHRS == AHRS_COMPLEMENTARY_FILTER)
		ahrs_estimate(&att_euler_est, ahrs.q, imu.accel_lpf, imu.gyro_lpf);
		ahrs.attitude.roll = att_euler_est.roll;
		ahrs.attitude.pitch = att_euler_est.pitch;
#elif (SELECT_AHRS ==  AHRS_MADGWICK_FILTER)
		madgwick_imu_ahrs(&madgwick_ahrs_info,
		                  imu.accel_lpf.x,
		                  imu.accel_lpf.y,
		                  imu.accel_lpf.z,
		                  deg_to_rad(imu.gyro_lpf.x),
		                  deg_to_rad(imu.gyro_lpf.y),
		                  deg_to_rad(imu.gyro_lpf.z));

		ahrs.attitude.roll = att_euler_est.roll = madgwick_ahrs_info.Roll;
		ahrs.attitude.pitch = att_euler_est.pitch = madgwick_ahrs_info.Pitch;

		ahrs.q[0] = madgwick_ahrs_info.q0;
		ahrs.q[1] = madgwick_ahrs_info.q1;
		ahrs.q[2] = madgwick_ahrs_info.q2;
		ahrs.q[3] = madgwick_ahrs_info.q3;
#endif

#if (SELECT_CONTROLLER == QUADROTOR_USE_PID)
		multirotor_pid_control(&imu, &ahrs, &rc);
#elif (SELECT_CONTROLLER == QUADROTOR_USE_GEOMETRY)
		float control_forces[3], control_moments[3];
		euler_t desired_attitude;
		desired_attitude.roll = deg_to_rad(-rc.roll);
		desired_attitude.pitch = deg_to_rad(-rc.pitch);
		desired_attitude.yaw = deg_to_rad(-rc.yaw);
		float gyro[3];
		gyro[0] = deg_to_rad(imu.gyro_lpf.x);
		gyro[1] = deg_to_rad(imu.gyro_lpf.y);
		gyro[2] = deg_to_rad(imu.gyro_lpf.z);
		float throttle_force = convert_motor_cmd_to_thrust(rc.throttle / 100.0f); //FIXME
		estimate_uav_dynamics(gyro, uav_dynamics_m, uav_dynamics_m_rot_frame);
		geometry_ctrl(&desired_attitude, ahrs.q, gyro, control_forces, control_moments);

		if(rc.safety == false) {
			led_on(LED_R);
			led_off(LED_B);
			thrust_allocate_quadrotor(motor_cmd, control_moments, throttle_force);
		} else {
			led_on(LED_B);
			led_off(LED_R);
			motor_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
#endif

		taskYIELD();
	}
}
