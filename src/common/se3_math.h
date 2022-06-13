#ifndef __SE3_MATH_H__
#define __SE3_MATH_H__

#include <math.h>

#define deg_to_rad(angle) (angle * M_PI / 180.0)
#define rad_to_deg(radian) (radian * 180.0 / M_PI)

#include "se3_math.h"

typedef struct {
	float roll;
	float pitch;
	float yaw;
} euler_t;

void euler_to_rotation_matrix(euler_t *euler, float *r, float *r_transpose);
void quat_to_rotation_matrix(float *q, float *r, float *r_transpose);
void vee_map_3x3(float *mat, float *vec);
void cross_product_3x1(float *vec_a, float *vec_b, float *vec_result);
void norm_3x1(float *vec, float *norm);
void normalize_3x1(float *vec);
float calc_vectors_angle_3x1(float *vec1, float *vec2);
void calc_matrix_multiply_vector_3d(float *vec_out, float *vec_in, float *matrix);

#endif
