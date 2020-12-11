#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>

#define OS_TICK 4000
#define freertos_task_delay(ms) vTaskDelay(OS_TICK / 1000 * ms)

void blocked_delay_ms(uint32_t ms);
void sys_timer_blocked_delay_ms(float delay_ms);

#endif
