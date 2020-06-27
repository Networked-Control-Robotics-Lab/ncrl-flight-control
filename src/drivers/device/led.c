#include "stm32f4xx.h"
#include "gpio.h"
#include "led.h"
#include "sys_time.h"

led_ctrl_t led_r;
led_ctrl_t led_g;
led_ctrl_t led_b;

int led_mode;

void rgb_led_init(void)
{
	led_r.gpio_group = GPIOA;
	led_r.pin_num = GPIO_Pin_2;
	led_r.state = LED_OFF;
	led_r.timer = 0;

	led_g.gpio_group = GPIOA;
	led_g.pin_num = GPIO_Pin_0;
	led_g.state = LED_OFF;
	led_g.timer = 0;

	led_b.gpio_group = GPIOA;
	led_b.pin_num = GPIO_Pin_3;
	led_b.state = LED_OFF;
	led_b.timer = 0;

	GPIO_ResetBits(led_r.gpio_group, led_r.pin_num);
	GPIO_ResetBits(led_g.gpio_group, led_g.pin_num);
	GPIO_ResetBits(led_b.gpio_group, led_b.pin_num);
}

void set_led_mode(int led_mode)
{
	switch(led_mode) {
	case LED_MODE_RC_PROTECTION:
		led_r.on_time = 0.1;
		led_r.off_time = 0.1;
		led_g.on_time = 0;
		led_g.off_time = 0;
		led_b.on_time = 0;
		led_b.off_time = 0;
		break;
	case LED_MODE_MOTOR_LOCKED:
		led_r.on_time = 0;
		led_r.off_time = 0;
		led_g.on_time = 0;
		led_g.off_time = 0;
		led_b.on_time = 1;
		led_b.off_time = 0;
		break;
	case LED_MODE_MOTOR_UNLOCKED:
		led_r.on_time = 1;
		led_r.off_time = 0;
		led_g.on_time = 0;
		led_g.off_time = 0;
		led_b.on_time = 0;
		led_b.off_time = 0;
		break;
	}
}

void led_control(led_ctrl_t *led, float curr_time)
{
	if((led->on_time == 0) && (led->off_time == 0)) {
		GPIO_ResetBits(led->gpio_group, led->pin_num);
		return;
	}

	if(led->state == LED_ON) {
		GPIO_SetBits(led->gpio_group, led->pin_num);
		if((curr_time - led->timer) >= led->on_time) {
			led->state = LED_OFF;
			led->timer = curr_time;
		}
	} else {
		GPIO_ResetBits(led->gpio_group, led->pin_num);
		if((curr_time - led->timer) >= led->off_time) {
			led->state = LED_ON;
			led->timer = curr_time;
		}
	}
}

void rgb_led_handler(void)
{
	float curr_time = get_sys_time_s();
	led_control(&led_r, curr_time);
	led_control(&led_g, curr_time);
	led_control(&led_b, curr_time);
}
