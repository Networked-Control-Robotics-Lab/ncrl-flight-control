#include "perf.h"
#include "sys_time.h"

perf_t *perf_ptr;
int perf_list_size;

void perf_init(perf_t *perf_list, int list_size)
{
	perf_ptr = perf_list;
	perf_list_size = list_size;
}

void perf_start(int id)
{
	perf_ptr[id].start_time = get_sys_time_s();
}

void perf_end(int id)
{
	perf_ptr[id].end_time = get_sys_time_s();
	perf_ptr[id].exec_time = perf_ptr[id].end_time - perf_ptr[id].start_time;
}

float perf_get_time_s(int id)
{
	return perf_ptr[id].exec_time;
}

char *perf_get_name(int id)
{
	return perf_ptr[id].name;
}

int perf_get_list_size(void)
{
	return perf_list_size;
}
