#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <stdbool.h>

#define WAYPOINT_NUM_MAX 50

enum {
	/* user manual flight mode */
	AUTOPILOT_MANUAL_FLIGHT_MODE,
	/* hovering at a waypoint */
	AUTOPILOT_HOVERING_MODE,
	/* follow the waypoint list to fly */
	AUTOPILOT_FOLLOW_WAYPOINT_MODE,
	/* hovering at current waypoint for some time before traveling to next one */
	AUTOPILOT_WAIT_NEXT_WAYPOINT_MODE,
	/* follow a trajectory by given position, velocity and acceleration */
	AUTOPILOT_TRAJECTORY_FOLLOWING_MODE,
	/* auto-takeoff mode */
	AUTOPILOT_TAKEOFF_MODE,
	/* auto-landing mode */
	AUTOPILOT_LANDING_MODE,
	/* motor locked mode */
	AUTOPILOT_MOTOR_LOCKED_MODE
} AUTOPILOT_MODE;

enum {
	AUTOPILOT_SET_SUCCEED,
	AUTOPILOT_WP_OUT_OF_FENCE,
	AUTOPILOT_WP_LIST_FULL,
	AUTOPILOT_MISSION_EXECUTING,
	AUTOPILOT_NO_EXECUTING_MISSION,
	AUTOPILOT_WP_LIST_EMPYT,
	AUTOPILOT_POSITION_NOT_FIXED,
	AUTOPILOT_UAV_ALREADY_TAKEOFF,
	AUTOPILOT_NOT_IN_HOVERING_MODE
} AUTOPILOT_SET_RETVAL;

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

/* every entities in autopilot_t is defined in enu frame */
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
	bool loop_mission;
	bool armed;

	struct waypoint_t wp_list[WAYPOINT_NUM_MAX]; //enu frame
	int curr_wp;
	int wp_num;
} autopilot_t;

void autopilot_init(autopilot_t *_autopilot);
void autopilot_update_uav_info(float pos_enu[3], float vel_enu[3]);
void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
void autopilot_set_mode(int new_mode);
void autopilot_set_armed(void);
void autopilot_set_disarmed(void);
bool autopilot_get_is_armed(void);
void autopilot_mission_reset(void);
int autopilot_get_mode(void);
int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius);
int autopilot_clear_waypoint_list(void);
int autopilot_goto_waypoint_now(float pos[3], bool change_height);
int autopilot_halt_waypoint_mission(void);
int autopilot_resume_waypoint_mission(void);
int autopilot_waypoint_mission_start(bool loop_mission);
int autopilot_trajectory_following_start(void);
int autopilot_trajectory_following_halt(void);
int autopilot_trigger_auto_landing(void);
int autopilot_trigger_auto_takeoff(void);
void autopilot_waypoint_handler(void);

void assign_vector_3x1_eun_to_ned(float *ned, float *enu);

void debug_print_waypoint_list(void);
void debug_print_waypoint_status(void);

#endif
