#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "arm_math.h"
#include "gpio.h"

#define MAT_ALLOC(mat, row, col) \
	arm_matrix_instance_f32 mat; \
	float mat ## _arr[row * col]

#define MAT_INIT(mat, row, col) \
	arm_mat_init_f32(&mat, row, col, (float32_t *)mat ## _arr)

#define MAT_ALLOC_INIT(mat, row, col) \
	arm_matrix_instance_f32 mat; \
	float mat ## _arr[row * col]; \
	arm_mat_init_f32(&mat, row, col, (float32_t *)mat ## _arr)

#define MAT_ASSERT(statement) \
	while(!(statement)) {led_on(LED_R);}

#define MAT_ADD(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_add_f32(mat_a, mat_b, mat_result); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define MAT_SUB(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_sub_f32(mat_a, mat_b, mat_result); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define MAT_MULT(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_mult_f32(mat_a, mat_b, mat_result); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define MAT_SCALE(mat_in, scale, mat_out) \
	mat_op_status = arm_mat_scale_f32(mat_in, scale, mat_out); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define MAT_TRANS(mat, mat_trans) \
	mat_op_status = arm_mat_trans_f32(mat, mat_trans); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define MAT_INV(mat, mat_inv) \
	mat_op_status = arm_mat_inverse_f32(mat, mat_inv); \
	MAT_ASSERT(mat_op_status == ARM_MATH_SUCCESS);

#define _mat_(mat) mat ## _arr

extern volatile arm_status mat_op_status;

#endif
