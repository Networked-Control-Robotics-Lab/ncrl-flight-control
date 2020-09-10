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

#define FLIGHT_CTL_PRESCALER_RELOAD 10

extern optitrack_t optitrack;

SemaphoreHandle_t flight_ctrl_semphr;

imu_t imu;
ahrs_t ahrs;
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

	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	do {
		time_current = get_sys_time_ms();
		if(time_current - time_last > 100.0f) {
			led_toggle(LED_R);
			time_last = time_current;
		}
		sbus_rc_read(&rc);
		vTaskDelay(1);

		//force to leave the loop if user triggered the motor esc range calibration
		if(is_esc_range_calibration_triggered() == true) return;

		//force to leave the loop if user triggered the motor thrust testing
		if(is_motor_force_testing_triggered() == true) return;
	} while(rc_safety_check(&rc) == 1);
}

void rc_yaw_setpoint_handler(float *desired_yaw, float rc_yaw_cmd, float dt)
{
	/* changing yaw setpoint if yaw joystick exceed the +-5 degree zone */
	if(rc_yaw_cmd > +5.0f || rc_yaw_cmd < -5.0f) {
		*desired_yaw += rc_yaw_cmd * dt;
		/* signal bounding */
		if(*desired_yaw > +180.0f) {
			*desired_yaw -= 360.0f;
		} else if(*desired_yaw < -180.0f) {
			*desired_yaw += 360.0f;
		}
	}
}

void task_flight_ctrl(void *param)
{
#if (SELECT_CONTROLLER == QUADROTOR_USE_PID)
	multirotor_pid_controller_init();
#elif (SELECT_CONTROLLER == QUADROTOR_USE_GEOMETRY)
	geometry_ctrl_init();
#endif

	mpu6500_init(&imu);

	rc_safety_protection();

	if(is_esc_range_calibration_triggered() == true) {
		esc_range_calibration();
	}

	if(is_motor_force_testing_triggered() == true) {
		motor_force_testing();
	}

	motor_init();

	float desired_yaw = 0.0f;

	while(imu_calibration_not_finished() == true) {
		led_on(LED_R);
		led_off(LED_G);
		led_on(LED_B);
		freertos_task_delay(2.5);
	}

	barometer_wait_until_stable();
	ahrs_init();

	led_off(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	while(1) {
		perf_start(PERF_FLIGHT_CONTROL_TRIGGER_TIME);
		while(xSemaphoreTake(flight_ctrl_semphr, 9) == pdFALSE);

		gpio_on(EXT_SW);
		perf_start(PERF_FLIGHT_CONTROL_LOOP);

#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG)
		ublox_m8n_gps_update();
#elif (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
		optitrack_update();
#endif

		sbus_rc_read(&rc);
		rc_yaw_setpoint_handler(&desired_yaw, -rc.yaw, 0.0025);

		perf_start(PERF_AHRS);
		float mag_raw[3]; //XXX
		get_imu_compass_raw(mag_raw);
		ahrs_estimate(&ahrs, imu.accel_lpf, imu.gyro_lpf, mag_raw);
		perf_end(PERF_AHRS);

		perf_start(PERF_CONTROLLER);
#if (SELECT_CONTROLLER == QUADROTOR_USE_PID)
		multirotor_pid_control(&imu, &ahrs, &rc, &desired_yaw);
#elif (SELECT_CONTROLLER == QUADROTOR_USE_GEOMETRY)
		multirotor_geometry_control(&imu, &ahrs, &rc, &desired_yaw);
#endif
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
	pack_debug_debug_message_header(payload, MESSAGE_ID_IMU);
	pack_debug_debug_message_float(&imu.accel_raw[0], payload);
	pack_debug_debug_message_float(&imu.accel_raw[1], payload);
	pack_debug_debug_message_float(&imu.accel_raw[2], payload);
	pack_debug_debug_message_float(&imu.accel_lpf[0], payload);
	pack_debug_debug_message_float(&imu.accel_lpf[1], payload);
	pack_debug_debug_message_float(&imu.accel_lpf[2], payload);
	pack_debug_debug_message_float(&imu.gyro_raw[0], payload);
	pack_debug_debug_message_float(&imu.gyro_raw[1], payload);
	pack_debug_debug_message_float(&imu.gyro_raw[2], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[0], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[1], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[2], payload);
	pack_debug_debug_message_float(&imu.temp_raw, payload);

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
	pack_debug_debug_message_float(&imu.accel_lpf[0], payload);
	pack_debug_debug_message_float(&imu.accel_lpf[1], payload);
	pack_debug_debug_message_float(&imu.accel_lpf[2], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[0], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[1], payload);
	pack_debug_debug_message_float(&imu.gyro_lpf[2], payload);
}
