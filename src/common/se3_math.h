#ifndef __SE3_MATH_H__
#define __SE3_MATH_H__

#include "se3_math.h"

void euler_to_rotation_matrix(euler_t *euler, float *r, float *r_transpose);
void quat_to_rotation_matrix(float *q, float *r, float *r_transpose);
void vee_map_3x3(float *mat, float *vec);
void cross_product_3x1(float *vec_a, float *vec_b, float *vec_result);
void norm_3x1(float *vec, float *norm);
void normalize_3x1(float *vec);

#endif
