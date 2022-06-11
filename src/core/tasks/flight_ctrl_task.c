#include <string.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "bound.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "sbus_radio.h"
#include "mpu6500.h"
#include "ms5611.h"
#include "motor.h"
#include "optitrack.h"
#include "lpf.h"
#include "imu.h"
#include "ahrs.h"
#include "multirotor_pid_ctrl.h"
#include "multirotor_geometry_ctrl.h"
#include "motor_thrust_fitting.h"
#include "flight_ctrl_task.h"
#include "sys_time.h"
#include "proj_config.h"
#include "debug_link.h"
#include "perf.h"
#include "perf_list.h"
#include "ublox_m8n.h"
#include "barometer.h"
#include "ist8310.h"
#include "esc_calibration.h"
#include "compass.h"
#include "vins_mono.h"
#include "ins.h"
#include "ins_sensor_sync.h"
#include "led.h"
#include "board_porting.h"
#include "system_state.h"

#define FLIGHT_CTL_PRESCALER_RELOAD 10

extern optitrack_t optitrack;
extern vins_mono_t vins_mono;

SemaphoreHandle_t flight_ctrl_semphr;

radio_t rc;

void flight_ctrl_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(flight_ctrl_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void rc_safety_protection(void)
{
	radio_t rc;

	float time_last = 0.0f;
	float time_current = 0.0f;

	set_rgb_led_rc_not_ready_flag(true);

	do {
		time_current = get_sys_time_ms();
		if(time_current - time_last > 100.0f) {
			time_last = time_current;
		}
		sbus_rc_read(&rc);
		vTaskDelay(1);

		//force to leave the loop if user triggered the motor esc range calibration
		if(is_esc_range_calibration_triggered() == true) break;

		//force to leave the loop if user triggered the motor thrust testing
		if(is_motor_force_testing_triggered() == true) break;
	} while(rc_safety_check(&rc) == 1);

	set_rgb_led_rc_not_ready_flag(false);
}

void task_flight_ctrl(void *param)
{
#if (SELECT_CONTROLLER == QUADROTOR_USE_PID)
	multirotor_pid_controller_init();
#elif (SELECT_CONTROLLER == QUADROTOR_USE_GEOMETRY)
	geometry_ctrl_init();
#endif

	/* imu initialization */
	imu_init();

	/* imu requires calibration before using */
	set_rgb_led_calibration_mode_flag(true);
	while(imu_calibration_not_finished() == true) {
		vTaskDelay(1);
	}
	set_rgb_led_calibration_mode_flag(false);

	/* blocked until user reset remote controller to safe position */
	rc_safety_protection();

	/* electronic speed controller calibration (triggered by qgroundcontrol or shell) */
	if(is_esc_range_calibration_triggered() == true) {
		/* since esc calibration is dangerous, once the program entered into
		 * the calibration process, it will never able to leave until the
		 * user reboot the system */
		esc_range_calibration();
	}

	/* motor force calibration (triggered by shell) */
	if(is_motor_force_testing_triggered() == true) {
		/* since motor force calibration is dangerous, once the program entered
		 * into the calibration process, it will never able to leave until the
		 * user reboot the system */
		motor_force_testing();
	}

	/* user did not triggered the calibration process, now we can sefely reset the
	 * motors to 0% thrust */
	motor_init();

	/* initialize barometer and compass */
	barometer_wait_until_stable();
	compass_wait_until_stable();

	/* ahrs and ins initialization */
	ahrs_init();
	ins_init();

	/* flight control loop */
	while(1) {
		perf_start(PERF_FLIGHT_CONTROL_TRIGGER_TIME);
		while(xSemaphoreTake(flight_ctrl_semphr, 9) == pdFALSE);

		gpio_on(EXT_SW);
		perf_start(PERF_FLIGHT_CONTROL_LOOP);

		/* sensor driver calls */
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS)
		ublox_m8n_gps_update();
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
		optitrack_update();
#endif

#if (SELECT_NAVIGATION_DEVICE2 == NAV_DEV2_USE_VINS_MONO)
		vins_mono_camera_trigger_20hz();
		vins_mono_send_imu_200hz();
		vins_mono_update();
#endif

		sbus_rc_read(&rc);

		/* attitude estimation */
		perf_start(PERF_AHRS_INS);
		{
			ins_state_estimate();
		}
		perf_end(PERF_AHRS_INS);

		/* controller */
		perf_start(PERF_CONTROLLER);
		{
#if (SELECT_CONTROLLER == QUADROTOR_USE_PID)
			multirotor_pid_control(&rc);
#elif (SELECT_CONTROLLER == QUADROTOR_USE_GEOMETRY)
			multirotor_geometry_control(&rc);
#endif
		}
		perf_end(PERF_CONTROLLER);

		perf_end(PERF_FLIGHT_CONTROL_LOOP);
		perf_end(PERF_FLIGHT_CONTROL_TRIGGER_TIME);
		gpio_off(EXT_SW);

		taskYIELD();
	}
}

void flight_controller_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	flight_ctrl_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_flight_ctrl, task_name, stack_size, NULL, priority, NULL);
}

void send_imu_debug_message(debug_msg_t *payload)
{
	float accel_raw[3], accel_lpf[3];
	get_accel_raw(accel_raw);
	get_accel_lpf(accel_lpf);

	float gyro_raw[3], gyro_lpf[3];
	get_gyro_raw(gyro_raw);
	get_gyro_lpf(gyro_lpf);

	float imu_temp_raw = get_imu_temperature();

	pack_debug_debug_message_header(payload, MESSAGE_ID_IMU);
	pack_debug_debug_message_float(&accel_raw[0], payload);
	pack_debug_debug_message_float(&accel_raw[1], payload);
	pack_debug_debug_message_float(&accel_raw[2], payload);
	pack_debug_debug_message_float(&accel_lpf[0], payload);
	pack_debug_debug_message_float(&accel_lpf[1], payload);
	pack_debug_debug_message_float(&accel_lpf[2], payload);
	pack_debug_debug_message_float(&gyro_raw[0], payload);
	pack_debug_debug_message_float(&gyro_raw[1], payload);
	pack_debug_debug_message_float(&gyro_raw[2], payload);
	pack_debug_debug_message_float(&gyro_lpf[0], payload);
	pack_debug_debug_message_float(&gyro_lpf[1], payload);
	pack_debug_debug_message_float(&gyro_lpf[2], payload);
	pack_debug_debug_message_float(&imu_temp_raw, payload);

}

void send_attitude_euler_debug_message(debug_msg_t *payload)
{
	float attitude_roll, attitude_pitch, attitude_yaw;
	get_attitude_euler_angles(&attitude_roll, &attitude_pitch, &attitude_yaw);

	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_EULER);
	pack_debug_debug_message_float(&attitude_roll, payload);
	pack_debug_debug_message_float(&attitude_pitch, payload);
	pack_debug_debug_message_float(&attitude_yaw, payload);
}

void send_attitude_quaternion_debug_message(debug_msg_t *payload)
{
	float attitude_q[4];
	get_attitude_quaternion(attitude_q);

	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_QUAT);
	pack_debug_debug_message_float(&attitude_q[0], payload);
	pack_debug_debug_message_float(&attitude_q[1], payload);
	pack_debug_debug_message_float(&attitude_q[2], payload);
	pack_debug_debug_message_float(&attitude_q[3], payload);
}

void send_attitude_imu_debug_message(debug_msg_t *payload)
{
	float accel_lpf[3];
	get_accel_lpf(accel_lpf);

	float gyro_lpf[3];
	get_gyro_lpf(gyro_lpf);

	float attitude_roll, attitude_pitch, attitude_yaw;
	get_attitude_euler_angles(&attitude_roll, &attitude_pitch, &attitude_yaw);

	pack_debug_debug_message_header(payload, MESSAGE_ID_ATTITUDE_IMU);
	pack_debug_debug_message_float(&attitude_roll, payload);
	pack_debug_debug_message_float(&attitude_pitch, payload);
	pack_debug_debug_message_float(&attitude_yaw, payload);
	pack_debug_debug_message_float(&accel_lpf[0], payload);
	pack_debug_debug_message_float(&accel_lpf[1], payload);
	pack_debug_debug_message_float(&accel_lpf[2], payload);
	pack_debug_debug_message_float(&gyro_lpf[0], payload);
	pack_debug_debug_message_float(&gyro_lpf[1], payload);
	pack_debug_debug_message_float(&gyro_lpf[2], payload);
}
