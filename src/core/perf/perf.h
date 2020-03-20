#ifndef __PERF_H__
#define __PERF_H__

#define DEF_PERF(id, name_str) [id] = {.name = name_str},
#define SIZE_OF_PERF_LIST(list) (sizeof(list) / sizeof(perf_t))

typedef struct {
	char *name;
	float start_time;
	float end_time;
	float exec_time;
} perf_t;

void perf_init(perf_t *perf_list, int list_size);
void perf_start(int id);
void perf_end(int id);
float perf_get_time_s(int id);
char *perf_get_name(int id);
int perf_get_list_size(void);

#endif
