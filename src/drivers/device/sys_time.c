#include <stdio.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "uart.h"
#include "sys_time.h"

sys_time_t sys_tim;

#define SYS_TIM_TICK_FREQ   400000
#define SYS_TIM_TICK_FREQ_MS   400
#define SYS_TIM_TICK_PERIOD (1.0f / SYS_TIM_TICK_FREQ)

void sys_time_update_handler(void)
{
	sys_tim.tick++;
	sys_tim.tick_s = SYS_TIM_TICK_PERIOD * (float)sys_tim.tick;

	if(sys_tim.tick == SYS_TIM_TICK_FREQ) {
		sys_tim.time_s += 1.0f;
		sys_tim.tick_s = 0.0f;
		sys_tim.tick = 0;
	}
	if(sys_tim.tick%SYS_TIM_TICK_FREQ_MS == 0) {
		sys_tim.tick_ms ++;
	}
}

float get_sys_time_ms(void)
{
	return (sys_tim.time_s + sys_tim.tick_s) * 1000.0f;
}

float get_sys_time_s(void)
{
	return sys_tim.time_s + sys_tim.tick_s;
}

uint32_t get_sys_tick_ms(void){
	return sys_tim.tick_ms;
}

void debug_print_sys_tim(void)
{
	char s[100] = {0};
	sprintf(s, "sys_tim_sec: %f, sys_time_ms: %f\n\r",
	        get_sys_time_s(), get_sys_time_ms());
	uart1_puts(s, strlen(s));
}
