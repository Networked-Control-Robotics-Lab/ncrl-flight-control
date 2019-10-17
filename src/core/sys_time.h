#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#include <stdint.h>

typedef struct {
	uint32_t tick;
	float tick_s;
	float time_s;
} sys_time_t;

void sys_time_update_handler(void);
void debug_print_sys_tim(void);

#endif
