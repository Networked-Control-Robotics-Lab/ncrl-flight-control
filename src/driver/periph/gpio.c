#include "stm32f4xx.h"
#include "gpio.h"

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

	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);
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
