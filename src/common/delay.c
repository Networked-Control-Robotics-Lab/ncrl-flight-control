#include <stdint.h>

void blocked_delay_ms(uint32_t ms)
{
	volatile uint32_t cnt;
	for(cnt = ms; cnt > 0; cnt--) {
		volatile uint32_t tweaked_delay = 22500U;
		while(tweaked_delay--);
	}
}
