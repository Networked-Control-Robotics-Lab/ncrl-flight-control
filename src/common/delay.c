#include <stdint.h>

#define STM32_CLOCK_HZ 180000000UL
#define STM32_CYCLES_PER_LOOP 5.5 // This will need tweaking or calculating

void blocked_delay_ms(uint32_t ms)
{
	ms *= STM32_CLOCK_HZ / 1000 / STM32_CYCLES_PER_LOOP;

	asm volatile(" mov r0, %[ms] \n\t"
	             "1: subs r0, #1 \n\t"
	             " bhi 1b \n\t"
	             :
	             : [ms] "r" (ms)
	             : "r0");

#if 0
	uint32_t cnt;
	for(cnt = ms; cnt > 0; cnt++) {
		uint32_t tweaked_delay = 1000000;
		while(tweaked_delay--);
	}
#endif
}
