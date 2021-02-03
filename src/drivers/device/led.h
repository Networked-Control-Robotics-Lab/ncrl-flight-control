#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>
#include "stm32f4xx.h"
#include "gpio.h"

enum {
	LED_OFF = 0,
	LED_ON =  1
} LED_STATE;

typedef struct {
	/* gpio pin */
	GPIO_TypeDef *gpio_group;
	uint16_t pin_num;

	uint8_t state;
} led_t;

typedef struct {
	bool service_enabled;

	struct {
		bool rc_protection;
		bool motor_lock;
		bool sensor_error;
		bool navigation_on;
	} flags;
} rgb_led_service_t;

void rgb_led_init(void);
void rgb_led_handler(void);

void enable_rgb_led_service(void);
void set_rgb_led_service_motor_lock_flag(bool new_state);
void set_rgb_led_service_sensor_error_flag(bool new_state);
void set_rgb_led_service_navigation_on_flag(bool new_state);

#endif
