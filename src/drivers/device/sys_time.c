#include <stdio.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "uart.h"
#include "sys_time.h"
#include "board_porting.h"

sys_time_t sys_tim;

#define SYS_TIM_TICK_FREQ   400000
#define SYS_TIM_TICK_PERIOD (1.0f / SYS_TIM_TICK_FREQ)

void sys_time_update_handler(void)
{
	sys_tim.tick++;

	if(sys_tim.tick == SYS_TIM_TICK_FREQ) {
		sys_tim.time_s++;
		sys_tim.tick = 0;
	}
}

float get_sys_time_ms(void)
{
	float residual = SYS_TIM_TICK_PERIOD * (float)sys_tim.tick;
	return (sys_tim.time_s + residual) * 1000.0f;
}

float get_sys_time_s(void)
{
	float residual = SYS_TIM_TICK_PERIOD * (float)sys_tim.tick;
	return sys_tim.time_s + residual;
}

/* use get_sys_time_tick() for accurate time control */
uint64_t get_sys_time_tick(void)
{
	return sys_tim.time_s * SYS_TIM_TICK_FREQ + sys_tim.tick;
}

void debug_print_sys_tim(void)
{
	char s[100] = {0};
	sprintf(s, "sys_tim_sec: %f, sys_time_ms: %f\n\r",
	        get_sys_time_s(), get_sys_time_ms());
	debug_link_puts(s, strlen(s));
}
