#ifndef __ATTITUDE_GEOMETRY_CTRL_H__
#define __ATTITUDE_GEOMETRY_CTRL_H__

void geometry_ctrl_init(void);
void geometry_ctrl(euler_t *rc, float attitude_q[4], float *output_forces, float *output_moments);
void thrust_allocate_quadrotor(float *motors, float *moments, float force_total);

#endif
