#ifndef __TRAJECTORY_FOLLOWING_H__
#define __TRAJECTORY_FOLLOWING_H__

void autopilot_assign_trajactory_waypoint(float time);
int autopilot_set_x_trajectory(int index, float *x_traj_coeff, float fligt_time);
int autopilot_set_y_trajectory(int index, float *y_traj_coeff, float fligt_time);
int autopilot_set_z_trajectory(int index, float *z_traj_coeff, float fligt_time);
int autopilot_set_yaw_trajectory(int index, float *yaw_traj_coeff, float fligt_time);
int autopilot_config_trajectory_following(int traj_num, bool z_traj, bool yaw_traj);
int autopilot_trajectory_following_start(bool loop_trajectory);
int autopilot_trajectory_following_stop(void);

void autopilot_trajectory_following_handler(void);

#endif
