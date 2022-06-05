#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f4xx.h"
#include <proj_config.h>

#define gpio_toggle(pin)  GPIO_ToggleBits(pin)
#define gpio_on(pin)  GPIO_SetBits(pin)
#define gpio_off(pin)  GPIO_ResetBits(pin)

void px4_board_gpio_config(void);

#endif
