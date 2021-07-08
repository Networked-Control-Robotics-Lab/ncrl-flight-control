#ifndef __SENSOR_SWITCHING_H__
#define __SENSOR_SWITCHING_H__

typedef struct {
	int curr_nav_sys;
	int new_nav_sys;
	bool require_update;
} nav_sys_manager_t;

enum {
	NAV_GNSS_INS,
	NAV_LOCAL_VIO,
	NAV_GNSS_ALIGNED_VIO
} NAV_SYS_SELECTION;

void navigation_system_manager_init(void);
void sensor_switching_handler(void);

void switch_navigation_system(int new_nav_sys);

#endif
