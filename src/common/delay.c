#include <stdint.h>
#include "sys_time.h"

void blocked_delay_ms(uint32_t ms)
{
	volatile uint32_t cnt;
	for(cnt = ms; cnt > 0; cnt--) {
		volatile uint32_t tweaked_delay = 22500U;
		while(tweaked_delay--);
	}
}

void sys_timer_blocked_delay_ms(float delay_ms)
{
	float start_time = get_sys_time_ms();
	float curr_time;

	do {
		curr_time = get_sys_time_ms();
	} while((curr_time - start_time) < delay_ms);
}

void sys_timer_blocked_delay_tick_ms(uint32_t delay_ms)
{
	uint32_t start_time = get_sys_tick_ms();
	while(1){
		if( ((get_sys_tick_ms() - start_time) > delay_ms) || (delay_ms == 0U)){
			break;
		}
	};
}
