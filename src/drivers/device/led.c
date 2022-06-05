#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "led.h"
#include "ncp5623c.h"
#include "sys_time.h"
#include "proj_config.h"
#include "board_support.h"

rgb_service_t rgb_service;

void rgb_led_init(void)
{
	rgb_service.red_on = false;
	rgb_service.green_on = false;
	rgb_service.blue_on = false;
}

void enable_rgb_led_service(void)
{
	rgb_led_init();
	rgb_service.service_enabled = true;
}

void config_rgb_mode(void)
{
	if(rgb_service.flags.navigation_on == true) {
		/* green on */
		rgb_service.green_on = true;
	} else {
		/* green off */
		rgb_service.green_on = false;
	}

	if(rgb_service.flags.motor_lock == true) {
		/* red static */
		rgb_service.red_on = false;
		rgb_service.blue_on = true;
	} else {
		if(rgb_service.flags.sensor_error == true) {
			/* pink static */
			rgb_service.red_on = true;
			rgb_service.green_on = true;
			rgb_service.blue_on = true;
		} else {
			/* blue static */
			rgb_service.red_on = true;
			rgb_service.blue_on = false;
		}
	}
}

void set_rgb_led_service_motor_lock_flag(bool new_state)
{
	rgb_service.flags.motor_lock = new_state;
	config_rgb_mode();
}

void set_rgb_led_service_sensor_error_flag(bool new_state)
{
	rgb_service.flags.sensor_error = new_state;
	config_rgb_mode();
}

void set_rgb_led_service_navigation_on_flag(bool new_state)
{
	rgb_service.flags.navigation_on = new_state;
	config_rgb_mode();
}

void rgb_led_handler(void)
{
	if(rgb_service.service_enabled == false) {
		return;
	}

	led_control(rgb_service.red_on,
	            rgb_service.green_on,
	            rgb_service.blue_on);
}
