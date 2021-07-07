#ifndef __SENSOR_SWITCHING_H__
#define __SENSOR_SWITCHING_H__

typedef struct {
	int select; //selection of the navigation system in use
} nav_sys_manager_t;

enum {
	NAV_GNSS_INS,
	NAV_LOCAL_VIO,
	NAV_GNSS_ALIGNED_VIO
} NAV_SYS_SELECTION;

#endif
