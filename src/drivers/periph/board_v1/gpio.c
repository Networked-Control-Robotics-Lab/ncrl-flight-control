#include <stdbool.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "board_porting.h"

/*
 * led_r: gpio_pin_2
 * led_g: gpio_pin_0
 * led_b: gpio_pin_3
 */
void led_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_ResetBits(LED_R);
	GPIO_ResetBits(LED_G);
	GPIO_ResetBits(LED_B);
}

/* external switch: pb14 */
void ext_switch_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_14,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void gpio_led_control(bool red_on, bool green_on, bool blue_on)
{
	if(red_on == true) {
		GPIO_SetBits(LED_R);
	} else {
		GPIO_ResetBits(LED_R);
	}

	if(green_on == true) {
		GPIO_SetBits(LED_G);
	} else {
		GPIO_ResetBits(LED_G);
	}

	if(blue_on == true) {
		GPIO_SetBits(LED_B);
	} else {
		GPIO_ResetBits(LED_B);
	}
}

void gpio_led_toggle(bool red_toggle, bool green_toggle, bool blue_toggle)
{
	if(red_toggle == true) {
		GPIO_ToggleBits(LED_R);
	}

	if(green_toggle == true) {
		GPIO_ToggleBits(LED_G);
	}

	if(blue_toggle == true) {
		GPIO_ToggleBits(LED_B);
	}
}
