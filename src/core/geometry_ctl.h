#ifndef __GEOMETRY_CTL_H__
#define __GEOMETRY_CTL_H__

#include "arm_math.h"
#include "madgwick_ahrs.h"

#define Kr1 240
#define Kr2 240
#define Kr3 300

#define Kg1 90
#define Kg2 90
#define Kg3 150

#define Ki1 3
#define Ki2 3
#define Ki3 3

#define Km 13

#define roll_middle 1553
#define pitch_middle 1552
#define yaw_middle 1550
#define throttle_max 1971
#define throttle_min 1127
#define throttle_middle 1548

typedef struct _controller_t {
	int pitch;
	int roll;
	int yaw;
	int throttle;
	int mode, mode_last;
	float yaw_command;
} futaba_t;

typedef struct _camera_t {
	int check;
	float time_period, time;
	float X, Y, Z, Yaw, Yaw_rad;
	float Rev_x, Rev_y, Rev_z, Rev_err_x, Rev_err_y, Rev_err_z, Rev_roll, Rev_pitch, Rev_yaw, Rev_throttle;
	float roll, pitch, err_x, err_y, err_z, throttle;
	float last_roll, last_pitch, last_throttle;
	float des_force_x, des_force_y, des_force_z, Rev_des_force_x, Rev_des_force_y, Rev_des_force_z;
} camera_t;

static float M_moment[9] = {
	0.02f,	0.00f,	0.00f,
	0.00f,	0.02f,	0.00f,
	0.00f,	0.00f,	0.03f,
};

float norm(float input_matrix[], int matrix_length);

void cross_product(float M_A[3], float M_B[3], float M_output[3]);

void Rotation_Matrix(float* roll, float* pitch, float* yaw, float* rotation_matrix, float* rotation_matrix_inv);
void Rotation_Matrix_Quaternion(float* q0, float* q1, float* q2, float* q3, float* rotation_matrix, float* rotation_matrix_inv);

void convert_to_hat_matrix(float* original_matrix, float* hat_matrix);


void inverse_hat_matrix(float* original_matrix, float* inverse_matrix);


void Geometric_Tracking_Control(MPU9250_t* MPU9250, Madgwick_t* Madgwick, camera_t* camera, float* thrust, float time_period);

void Geometric_Tracking_Control_Manual(MPU9250_t* MPU9250, Madgwick_t* Madgwick, camera_t* camera, futaba_t* futaba, float* thrust, float time_period);

#endif
