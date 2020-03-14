#include "perf.h"
#include "sys_time.h"

perf_t *perf_ptr;

void perf_init(perf_t *perf_arr)
{
	perf_ptr = perf_arr;
}

void perf_start(int id)
{
	perf_ptr[id].start_time = get_sys_time_ms();
}

void perf_end(int id)
{
	perf_ptr[id].end_time = get_sys_time_ms();
	perf_ptr[id].exec_time = perf_ptr[id].end_time - perf_ptr[id].start_time;
}

float perf_get_time_ms(int id)
{
	return perf_ptr[id].exec_time;
}
