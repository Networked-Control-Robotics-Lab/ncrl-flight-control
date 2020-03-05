#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include <stdbool.h>

#define WAYPOINT_NUM_MAX 50

enum {
	/* hovering at a waypoint */
	NAV_HOVERING_WAYPOINT,
	/* follow the waypoint list to fly */
	NAV_FOLLOW_WAYPOINT,
	/* hovering by current waypoint for some time before traveling to next one */
	NAV_WAIT_NEXT_WAYPOINT,
	/* follow a trajectory by given position, velocity and acceleration */
	NAV_TRAJECTORY_FOLLOWING_MODE
} NAV_MODE;

enum {
	WP_SET_SUCCEED,
	WP_SET_OUT_OF_FENCE,
	WP_SET_EXCEED_MAX_WP,
	WP_NO_EXECUTING_WP_LIST,
	WP_WP_LIST_EXECUTING,
	WP_WP_LIST_EMPTY
} WP_SET_RETVAL;

struct waypoint_t {
	float pos[3];        //[m]
	float heading;       //[deg]
	float halt_time_sec; //[s]
	float touch_radius;  //[m]
};

typedef struct {
	struct waypoint_t wp_now; //enu frame

	//earth-north-up
	struct {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence; /* rectangular geo-fence in enu frame */

	int mode;
	bool halt_flag;

	struct waypoint_t wp_list[WAYPOINT_NUM_MAX]; //enu frame
	int curr_wp;
	int wp_num;
} nav_t;

void nav_init(nav_t *_nav);
void nav_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
int nav_add_new_waypoint(float pos[3], float heading, float halt_time_sec);
int nav_clear_waypoint_list(void);
int nav_goto_waypoint_now(float pos[3], bool change_height);
int nav_halt_waypoint_mission(void);
int nav_resume_waypoint_mission(void);
int nav_waypoint_mission_start(void);
void nav_waypoint_handler(float curr_pos[3]);

#endif
