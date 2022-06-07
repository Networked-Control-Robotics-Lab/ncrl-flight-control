#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#include <stdint.h>

typedef struct {
	uint32_t tick;
	uint32_t time_s;
} sys_time_t;

void sys_time_update_handler(void);
float get_sys_time_ms(void);
float get_sys_time_s(void);
uint64_t get_sys_time_tick(void);
void debug_print_sys_tim(void);

#endif
