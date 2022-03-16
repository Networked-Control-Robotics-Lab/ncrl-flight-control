#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#include <stdint.h>

typedef struct {
	uint32_t tick;
	float tick_s;
	float time_s;
	
	uint32_t tick_ms;	//overflow about 49days
} sys_time_t;

void sys_time_update_handler(void);
float get_sys_time_ms(void);
float get_sys_time_s(void);
uint32_t get_sys_tick_ms(void);
void debug_print_sys_tim(void);

#endif
