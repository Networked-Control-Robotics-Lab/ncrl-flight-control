#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "se3_math.h"

void quaternion_copy(float *q_dest, float *q_src);
void quaternion_mult(float *q1, float *q2, float *q_mult);
void quaternion_conj(float *q, float *q_conj);
void quat_normalize(float *q);
void quat_to_euler(float *q, euler_t *euler);
void euler_to_quat(euler_t *euler, float *q);

#endif
