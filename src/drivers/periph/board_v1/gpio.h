#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdbool.h>
#include "stm32f4xx.h"

#define EXT_SW GPIOB, GPIO_Pin_14
#define MOTOR8 GPIOE, GPIO_Pin_9

#define gpio_toggle(pin)  GPIO_ToggleBits(pin)
#define gpio_on(pin)  GPIO_SetBits(pin)
#define gpio_off(pin)  GPIO_ResetBits(pin)

#define LED_R GPIOA, GPIO_Pin_2
#define LED_G GPIOA, GPIO_Pin_0
#define LED_B GPIOA, GPIO_Pin_3

void led_init();
void ext_switch_init(void);

void gpio_led_control(bool red_on, bool green_on, bool blue_on);
void gpio_led_toggle(bool red_toggle, bool green_toggle, bool blue_toggle);

#endif
