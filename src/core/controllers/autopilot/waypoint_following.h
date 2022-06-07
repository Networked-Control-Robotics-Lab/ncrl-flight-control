#ifndef __WAYPOINT_FOLLOWING_H__
#define __WAYPOINT_FOLLOWING_H__

int autopilot_get_waypoint_count(void);
bool autopilot_get_waypoint_gps_mavlink(int index, int32_t *latitude, int32_t *longitude, float *height, uint16_t *cmd);
int autopilot_add_new_waypoint(float pos[3], float heading, float halt_time_sec);
int autopilot_add_new_waypoint_gps_mavlink(int frame, int32_t x, int32_t y, float z, uint16_t cmd);
int autopilot_clear_waypoint_list(void);
int autopilot_goto_waypoint_now(float yaw, float pos[3], bool change_height);
int autopilot_halt_waypoint_mission(void);
int autopilot_resume_waypoint_mission(void);
int autopilot_waypoint_mission_start(bool loop_mission);
void autopilot_mission_reset(void);

void autopilot_wait_next_waypoint_handler(void);
void autopilot_follow_waypoint_handler(float *curr_pos);
void autopilot_goto_handler(float *curr_pos);

#endif
