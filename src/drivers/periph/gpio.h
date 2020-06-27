#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f4xx.h"

#define EXT_SW GPIOB, GPIO_Pin_14

#define led_toggle(led)  GPIO_ToggleBits(led)
#define led_on(led)  GPIO_SetBits(led)
#define led_off(led)  GPIO_ResetBits(led)

#define gpio_toggle(pin)  GPIO_ToggleBits(pin)
#define gpio_on(pin)  GPIO_SetBits(pin)
#define gpio_off(pin)  GPIO_ResetBits(pin)

#define LED_R GPIOA, GPIO_Pin_2
#define LED_G GPIOA, GPIO_Pin_0
#define LED_B GPIOA, GPIO_Pin_3

void led_init();
void ext_switch_init(void);

#endif
