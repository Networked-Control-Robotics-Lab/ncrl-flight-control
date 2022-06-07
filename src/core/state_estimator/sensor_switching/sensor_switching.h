#ifndef __SENSOR_SWITCHING_H__
#define __SENSOR_SWITCHING_H__

typedef struct {
	int current_system;
	int new_system;
	bool require_update;
} navigation_manager_t;

enum {
	NAV_GNSS_INS,
	NAV_LOCAL_VIO,
	NAV_GLOBAL_VIO
} NAV_SYS_SELECTION;

void navigation_manager_init(void);
void navigation_manager_handler(void);

void switch_navigation_system(int new_system);

#endif
