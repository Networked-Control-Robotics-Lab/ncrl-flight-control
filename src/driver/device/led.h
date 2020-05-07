#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"
#include "gpio.h"

enum {
	LED_OFF,
	LED_ON
} LED_STATE;

enum {
        LED_MODE_RC_PROTECTION,
        LED_MODE_MOTOR_LOCKED,
	LED_MODE_MOTOR_UNLOCKED
};

typedef struct {
	/* gpio pin */
	GPIO_TypeDef *gpio_group;
	uint16_t pin_num;

	float on_time;
	float off_time;

	int state;

	float timer;
} led_ctrl_t;

void rgb_led_init(void);
void rgb_led_handler(void);
void set_led_mode(int led_mode);

#endif
