#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>
#include "stm32f4xx.h"
#include "gpio.h"

typedef struct {
	bool blink_enabled;
	float blink_start_time;
	float blink_wait_time;

	bool red_on;
	bool green_on;
	bool blue_on;

	/* flags that decide light pattern */
	bool calibration_mode;
	bool rc_not_ready;
	bool motor_lock;
	bool sensor_error;
	bool navigation_on;
} rgb_service_t;

void rgb_led_service_init(void);
void rgb_led_handler(void);

void set_rgb_led_service_motor_lock_flag(bool state);
void set_rgb_led_service_sensor_error_flag(bool state);
void set_rgb_led_service_navigation_on_flag(bool state);
void set_rgb_led_calibration_mode_flag(bool state);
void set_rgb_led_rc_not_ready_flag(bool state);

#endif
