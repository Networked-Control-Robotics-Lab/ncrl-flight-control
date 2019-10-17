#include <stdint.h>

#define STM32_CLOCK_HZ 180000000UL
#define STM32_CYCLES_PER_LOOP 5.5 // This will need tweaking or calculating

void blocked_delay_ms(uint32_t ms)
{
	volatile uint32_t cnt;
	for(cnt = ms; cnt > 0; cnt--) {
		volatile uint32_t tweaked_delay = 22500U;
		while(tweaked_delay--);
	}
}
