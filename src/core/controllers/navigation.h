#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include <stdbool.h>

#define WAYPOINT_NUM_MAX 50

enum {
	/* user manual flight mode */
	NAV_MANUAL_FLIGHT_MODE,
	/* hovering at a waypoint */
	NAV_HOVERING_MODE,
	/* follow the waypoint list to fly */
	NAV_FOLLOW_WAYPOINT_MODE,
	/* hovering at current waypoint for some time before traveling to next one */
	NAV_WAIT_NEXT_WAYPOINT_MODE,
	/* follow a trajectory by given position, velocity and acceleration */
	NAV_TRAJECTORY_FOLLOWING_MODE,
	/* auto-takeoff mode */
	NAV_TAKEOFF_MODE,
	/* auto-landing mode */
	NAV_LANDING_MODE,
	/* motor locked mode */
	NAV_MOTOR_LOCKED_MODE
} NAV_MODE;

enum {
	NAV_SET_SUCCEED,
	NAV_WP_OUT_OF_FENCE,
	NAV_WP_LIST_FULL,
	NAV_MISSION_EXECUTING,
	NAV_NO_EXECUTING_MISSION,
	NAV_WP_LIST_EMPYT,
	NAV_POSITION_NOT_FIXED,
	NAV_UAV_ALREADY_TAKEOFF
} WP_SET_RETVAL;

struct waypoint_t {
	float pos[3];        //[m]
	float heading;       //[deg]
	float halt_time_sec; //[s]
	float touch_radius;  //[m]
};

struct uav_info_t {
	float pos[3];
	float vel[3];
};

/* every entities in nav_t is defined in enu frame */
typedef struct {
	struct uav_info_t uav_info;
	struct waypoint_t wp_now;

	struct {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence; /* rectangular geo-fence in enu frame */

	float landing_speed;
	float landing_accept_height;
	float takeoff_speed;
	float takeoff_height;
	
	int mode;
	bool halt_flag;

	struct waypoint_t wp_list[WAYPOINT_NUM_MAX]; //enu frame
	int curr_wp;
	int wp_num;
} nav_t;

void nav_init(nav_t *_nav);
void nav_update_uav_info(float pos[3], float vel[3]);
void nav_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
int nav_add_new_waypoint(float pos[3], float heading, float halt_time_sec);
int nav_clear_waypoint_list(void);
int nav_goto_waypoint_now(float pos[3], bool change_height);
int nav_halt_waypoint_mission(void);
int nav_resume_waypoint_mission(void);
int nav_waypoint_mission_start(void);
int nav_trigger_auto_landing(void);
int nav_trigger_auto_takeoff(void);
void nav_waypoint_handler(void);

#endif
