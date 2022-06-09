#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "led.h"
#include "ncp5623c.h"
#include "sys_time.h"
#include "proj_config.h"
#include "board_porting.h"

rgb_service_t rgb_service;

void rgb_led_service_init(void)
{
	rgb_service.red_on = false;
	rgb_service.green_on = false;
	rgb_service.blue_on = false;
}

void config_rgb_mode(void)
{
	if(rgb_service.calibration_mode == true) {
		/* purple on */
		rgb_service.red_on = true;
		rgb_service.green_on = false;
		rgb_service.blue_on = true;

		rgb_service.blink_enabled = false;
		rgb_service.blink_wait_time = 0.0f;

		return;
	} else if(rgb_service.rc_not_ready) {
		/* blink red */
		rgb_service.red_on = true;
		rgb_service.green_on = false;
		rgb_service.blue_on = false;

		rgb_service.blink_start_time = get_sys_time_s();
		rgb_service.blink_enabled = true;
		rgb_service.blink_wait_time = 0.075; //[s]

		return;
	} else if(rgb_service.sensor_error == true) {
		/* pink on */
		rgb_service.red_on = true;
		rgb_service.green_on = true;
		rgb_service.blue_on = true;

		rgb_service.blink_enabled = false;
		rgb_service.blink_wait_time = 0.0f;

		return;
	}

	if(rgb_service.motor_lock == true) {
		/* blue on (disarmed) */
		rgb_service.red_on = false;
		rgb_service.blue_on = true;

		/* light red (navigation on) / dark red (navigation off) */
		rgb_service.green_on = (rgb_service.navigation_on) ? true : false;

		rgb_service.blink_enabled = false;
		rgb_service.blink_wait_time = 0.0f;

		return;
	} else {
		/* red on (armed) */
		rgb_service.red_on = true;
		rgb_service.blue_on = false;

		/* light red (navigation on) / dark red (navigation off) */
		rgb_service.green_on = (rgb_service.navigation_on) ? true : false;

		rgb_service.blink_enabled = false;
		rgb_service.blink_wait_time = 0.0f;

		return;
	}
}

void set_rgb_led_service_motor_lock_flag(bool state)
{
	rgb_service.motor_lock = state;
	config_rgb_mode();
}

void set_rgb_led_service_sensor_error_flag(bool state)
{
	rgb_service.sensor_error = state;
	config_rgb_mode();
}

void set_rgb_led_service_navigation_on_flag(bool state)
{
	rgb_service.navigation_on = state;
	config_rgb_mode();
}

void set_rgb_led_calibration_mode_flag(bool state)
{
	rgb_service.calibration_mode = state;
	config_rgb_mode();
}

void set_rgb_led_rc_not_ready_flag(bool state)
{
	rgb_service.rc_not_ready = state;
	config_rgb_mode();
}

void rgb_led_handler(void)
{
	if(rgb_service.blink_enabled == true) {
		/* calculate blink elapsed time */
		float curr_time = get_sys_time_s();
		float blink_elapsed_time = curr_time - rgb_service.blink_start_time;

		/* time's up, do toggle */
		if(blink_elapsed_time > rgb_service.blink_wait_time) {
			led_toggle(rgb_service.red_on, rgb_service.green_on, rgb_service.blue_on);
			rgb_service.blink_start_time = curr_time;
		}
	} else {
		/* pure on and off */
		led_control(rgb_service.red_on, rgb_service.green_on, rgb_service.blue_on);
	}
}
