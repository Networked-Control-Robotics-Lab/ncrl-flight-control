#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <stdbool.h>

#define TRAJ_WP_MAX_NUM 50

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
	AUTOPILOT_WAYPOINT_OUT_OF_FENCE,
	AUTOPILOT_WAYPOINT_LIST_FULL,
	AUTOPILOT_WAYPOINT_FOLLOWING_BUSY,
	AUTOPILOT_WAYPOINT_LIST_EMPYT,
	AUTOPILOT_NO_HALTED_WAYPOINT_MISSION,
	AUTOPILOT_TRAJACTORY_LIST_EMPTY,
	AUTOPILOT_TRAJACTORY_LIST_FULL,
	AUTOPILOT_TRAJACTORY_LIST_TOO_LARGE,
	AUTOPILOT_TRAJACTORY_FOLLOWING_BUSY,
	AUTOPILOT_NOT_IN_HOVERING_MODE,
	AUTOPILOT_NOT_IN_WAYPOINT_MODE,
	AUTOPILOT_NOT_IN_TRAJECTORY_MODE,
	AUTOPILOT_ALREADY_TAKEOFF
} AUTOPILOT_RETVAL;

struct trajectory_segment_t
{
	float x_poly_coeff[8];
	float y_poly_coeff[8];
	float z_poly_coeff[8];

	float vx_poly_coeff[7];
	float vy_poly_coeff[7];
	float vz_poly_coeff[7];

	float ax_poly_coeff[7];
	float ay_poly_coeff[7];
	float az_poly_coeff[7];

	float yaw_poly_coeff[4];
	float yaw_rate_poly_coeff[3];

	float flight_time;
};

struct waypoint_t {
	float pos[3];        //[m]
	float heading;       //[deg]
	float halt_time_sec; //[s]
	float touch_radius;  //[m]

	/* compatible with mavlink design */
	int32_t latitude;  //[deg], scaled by 1/1e7
	int32_t longitude; //[deg], scaled by 1/1e7
	float height;      //[m]
	uint16_t command;  //check MAV_CMD enum
};

/* every entities in autopilot_t is defined in enu frame */
typedef struct {
	struct {
		float pos[3];
		float vel[3];
	} uav_state; /* current position and velocity of the uav */

	struct {
		float pos[3];             //[m]
		float vel[3];             //[m/s]
		float acc_feedforward[3]; //[m/s^2]
		float heading;            //[deg]
	} wp_now; /* autopilot provides these to controller as desired setpoint */

	struct {
		float origin[3];
		float lx;
		float ly;
		float height;
	} geo_fence; /* rectangular geo-fence in enu frame */

	float landing_speed;
	float landing_accept_height_upper;
	float landing_accept_height_lower;
	bool land_avaliable;
	float takeoff_speed;
	float takeoff_height;
	
	int mode;
	bool halt_flag;
	bool loop_mission;
	bool armed;
	bool motor_locked;

	/* for waypoint following (representing setpoint with waypoints) */
	struct waypoint_t wp_list[TRAJ_WP_MAX_NUM]; //enu frame
	int curr_wp; //waypoint index, indicates which waypoint to track
	int wp_num;  //total waypoint number

	/* for trajectory following (representing setpoint with 7th ordered polynomials) */
	struct trajectory_segment_t trajectory_segments[TRAJ_WP_MAX_NUM];
	float trajectory_update_time;
	int curr_traj;  //trajectory segment index, indicates which trajectory to track
	int traj_num;   //total trajectory number
	float traj_start_time;
	bool z_traj;
	bool yaw_traj;
} autopilot_t;

bool check_motor_lock_condition(bool condition);

void autopilot_init(autopilot_t *_autopilot);
void autopilot_update_uav_state(float pos_enu[3], float vel_enu[3]);

bool autopilot_is_manual_flight_mode(void);
bool autopilot_is_motor_locked_mode(void);
bool autopilot_is_auto_flight_mode(void);

void autopilot_set_enu_rectangular_fence(float origin[3], float lx, float ly, float height);
void autopilot_set_mode(int new_mode);
void autopilot_set_armed(void);
void autopilot_set_disarmed(void);
void autopilot_mission_reset(void);

bool autopilot_is_armed(void);
int autopilot_get_mode(void);
void autopilot_get_pos_setpoint(float *pos_set);
void autopilot_get_vel_setpoint(float *vel_set);
void autopilot_get_accel_feedforward(float *accel_ff);

int autopilot_get_waypoint_count(void);
bool autopilot_get_waypoint_gps_mavlink(int index, int32_t *latitude, int32_t *longitude,
                                float *height, uint16_t *cmd);
int autopilot_add_new_waypoint_gps_mavlink(int32_t latitude, int32_t longitude,
                                     float height, uint16_t cmd);

int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time);
int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time);
int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time);
int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time);
int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj);
int autopilot_trajectory_following_start(bool loop_trajectory);
int autopilot_trajectory_following_stop(void);

int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec, float radius);

int autopilot_clear_waypoint_list(void);
int autopilot_goto_waypoint_now(float pos[3], bool change_height);
int autopilot_halt_waypoint_mission(void);
int autopilot_resume_waypoint_mission(void);
int autopilot_waypoint_mission_start(bool loop_mission);
int autopilot_trigger_auto_landing(void);
int autopilot_trigger_auto_takeoff(void);

void autopilot_lock_motor(void);
void autopilot_unlock_motor(void);

void autopilot_guidance_handler(void);

void assign_vector_3x1_eun_to_ned(float *ned, float *enu);

void debug_print_waypoint_list(void);
void debug_print_waypoint_status(void);

#endif
