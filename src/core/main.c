#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <proj_config.h>
#include <timer.h>
#include <sys_time.h>
#include <delay.h>
#include "ncp5623c.h"
#include "i2c.h"
#include "led.h"
void init_GPIOE()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}


int main()
{
	Init_I2C();
	init_GPIOE();
	timer3_init();
	timer12_init();
	enable_rgb_led_service();
	sys_timer_blocked_delay_tick_ms(50);
	int flag = 0;
	while(1) {
		if (flag == 0) {
			set_rgb_led_service_motor_lock_flag(true);
		} else if (flag == 1) {
			set_rgb_led_service_navigation_on_flag(true);
		} else if (flag == 2) {
			set_rgb_led_service_motor_lock_flag(false);
		} else if (flag == 3) {
			set_rgb_led_service_navigation_on_flag(false);
		}
		sys_timer_blocked_delay_tick_ms(1000);
		flag ++;
		flag %= 4;
	}

	return 0;
}
