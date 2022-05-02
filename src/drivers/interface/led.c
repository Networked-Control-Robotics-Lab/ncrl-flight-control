#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "led.h"
#include "ncp5623c.h"
#include "sys_time.h"
#include "proj_config.h"

led_t led_r;
led_t led_g;
led_t led_b;

rgb_led_service_t rgb_led_service;

void rgb_led_init(void)
{
#if (RGB_USE == RGB_USE_GPIO)
	led_r.gpio_group = GPIOA;
	led_r.pin_num = GPIO_Pin_2;
	led_r.state = LED_OFF;

	led_g.gpio_group = GPIOA;
	led_g.pin_num = GPIO_Pin_0;
	led_g.state = LED_OFF;

	led_b.gpio_group = GPIOA;
	led_b.pin_num = GPIO_Pin_3;
	led_b.state = LED_OFF;

#elif (RGB_USE == RGB_USE_NCP5623C)

	led_r.gpio_group = GPIOA;
	led_r.pin_num = GPIO_Pin_2;
	led_r.state = LED_OFF;

	led_g.gpio_group = GPIOA;
	led_g.pin_num = GPIO_Pin_0;
	led_g.state = LED_OFF;

	led_b.gpio_group = GPIOA;
	led_b.pin_num = GPIO_Pin_3;
	led_b.state = LED_OFF;

#endif
}

void rgb_led_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(rgb_led_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void task_rgb_led(void *param)
{	
	while(1) {
		while(xSemaphoreTake(rgb_led_semphr, 9) == pdFALSE);
		rgb_led_handler();
	}
}
void rgb_led_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	rgb_led_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_rgb_led, task_name, stack_size, NULL, priority, NULL);
}
void enable_rgb_led_service(void)
{
	rgb_led_init();
	rgb_led_service.service_enabled = true;
}

void config_rgb_mode(void)
{
	if(rgb_led_service.flags.navigation_on == true) {
		/* green on */
		led_g.state = LED_ON;
	} else {
		/* green off */
		led_g.state = LED_OFF;
	}

	if(rgb_led_service.flags.motor_lock == true) {
		/* red static */
		led_r.state = LED_OFF;
		led_b.state = LED_ON;
	} else {
		if(rgb_led_service.flags.sensor_error == true) {
			/* pink static */
			led_r.state = LED_ON;
			led_g.state = LED_ON;
			led_b.state = LED_ON;
		} else {
			/* blue static */
			led_r.state = LED_ON;
			led_b.state = LED_OFF;
		}
	}
}

void set_rgb_led_service_motor_lock_flag(bool new_state)
{
	rgb_led_service.flags.motor_lock = new_state;
	config_rgb_mode();
}

void set_rgb_led_service_sensor_error_flag(bool new_state)
{
	rgb_led_service.flags.sensor_error = new_state;
	config_rgb_mode();
}

void set_rgb_led_service_navigation_on_flag(bool new_state)
{
	rgb_led_service.flags.navigation_on = new_state;
	config_rgb_mode();
}

#if (RGB_USE == RGB_USE_GPIO)
void led_control(led_t *led)
{
	if(led->state == LED_ON) {
		GPIO_SetBits(led->gpio_group, led->pin_num);
	} else {
		GPIO_ResetBits(led->gpio_group, led->pin_num);
	}
}

#elif (RGB_USE == RGB_USE_NCP5623C)
void led_control(led_t *led_r,led_t *led_g,led_t *led_b)
{
	float r = 0;
	float g = 0;
	float b = 0;
	if(led_r->state == LED_ON) {
		r = 100.0f;
	}
	if(led_g->state == LED_ON) {
		g = 100.0f;
	}
	if(led_b->state == LED_ON) {
		b = 100.0f;
	}
	rgb_light(r,g,b);
}
#endif
void rgb_led_handler(void)
{
	if(rgb_led_service.service_enabled == false) {
		return;
	}

#if (RGB_USE == RGB_USE_GPIO)
	led_control(&led_r);
	led_control(&led_g);
	led_control(&led_b);
#elif (RGB_USE == RGB_USE_NCP5623C)
	led_control(&led_r,&led_g,&led_b);
#endif
}
