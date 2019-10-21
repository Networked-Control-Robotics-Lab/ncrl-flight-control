#include "geometry_ctl.h"
#include "madgwick_ahrs.h"

static float32_t Yaw;

static float32_t last_M_R_des[9] = {
	0, 0, 0,
	0, 0, 0,
	0, 0, 0,
};

static float32_t last_M_ang_vel_des[3] = {
	0,
	0,
	0,
};

static float32_t M_ang_vel_des_dot[3] = {
	0,
	0,
	0,
};

float32_t M_ang_vel_des[3] = {0, 0, 0}, M_accum_attitude_error[3] = {0, 0, 0};

float norm(float input_matrix[], int matrix_length)
{
	float Result;
	arm_power_f32(input_matrix, matrix_length, &Result);
	return sqrt(Result);
}

void cross_product(float M_A[3], float M_B[3], float M_output[3])
{
	float M_A_h[9];

	arm_matrix_instance_f32 A_h, B, output;

	arm_mat_init_f32(&A_h, 3, 3, M_A_h);
	arm_mat_init_f32(&B, 3, 1, M_B);
	arm_mat_init_f32(&output, 3, 1, M_output);

	convert_to_hat_matrix(M_A, M_A_h);
	arm_mat_mult_f32(&A_h, &B, &output);
}

void Rotation_Matrix(float* roll, float* pitch, float* yaw, float* rotation_matrix, float* rotation_matrix_inv)
{
	arm_matrix_instance_f32 M_R, M_R_I;

	rotation_matrix[0] = arm_cos_f32(*pitch)*arm_cos_f32(*yaw);
	rotation_matrix[1] = -arm_sin_f32(*yaw)*arm_cos_f32(*pitch);
	rotation_matrix[2] = arm_sin_f32(*pitch);
	rotation_matrix[3] = arm_cos_f32(*roll)*arm_sin_f32(*yaw)+arm_sin_f32(*pitch)*arm_sin_f32(*yaw)*arm_cos_f32(*roll);
	rotation_matrix[4] = arm_cos_f32(*roll)*arm_cos_f32(*yaw)-arm_sin_f32(*pitch)*arm_sin_f32(*yaw)*arm_sin_f32(*roll);
	rotation_matrix[5] = -arm_sin_f32(*roll)*arm_cos_f32(*pitch);
	rotation_matrix[6] = arm_sin_f32(*roll)*arm_sin_f32(*yaw)-arm_cos_f32(*roll)*arm_cos_f32(*yaw)*arm_sin_f32(*pitch);
	rotation_matrix[7] = arm_sin_f32(*roll)*arm_cos_f32(*yaw)+arm_cos_f32(*roll)*arm_sin_f32(*pitch)*arm_sin_f32(*yaw);
	rotation_matrix[8] = arm_cos_f32(*roll)*arm_cos_f32(*pitch);

	arm_mat_init_f32(&M_R, 3, 3, rotation_matrix);
	arm_mat_init_f32(&M_R_I, 3, 3, rotation_matrix_inv);
	arm_mat_trans_f32(&M_R, &M_R_I);
}

void Rotation_Matrix_Quaternion(float* q0, float* q1, float* q2, float* q3, float* rotation_matrix, float* rotation_matrix_inv)
{
	arm_matrix_instance_f32 M_R, M_R_I;

	float q0q0 = *q0 * *q0;
	float q1q1 = *q1 * *q1;
	float q2q2 = *q2 * *q2;
	float q3q3 = *q3 * *q3;
	float q0q1 = *q0 * *q1;
	float q0q2 = *q0 * *q2;
	float q0q3 = *q0 * *q3;
	float q1q2 = *q1 * *q2;
	float q1q3 = *q1 * *q3;
	float q2q3 = *q2 * *q3;
	float q1q1_q2q2 = q1q1 + q2q2;

	rotation_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;
	rotation_matrix[1] = 2*(q1q2 + q0q3);
	rotation_matrix[2] = 2*(q1q3 - q0q2);
	rotation_matrix[3] = 2*(q1q2 - q0q3);
	rotation_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;
	rotation_matrix[5] = 2*(q2q3 + q0q1);
	rotation_matrix[6] = 2*(q1q3 + q0q2);
	rotation_matrix[7] = 2*(q2q3 - q0q1);
	rotation_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;

	arm_mat_init_f32(&M_R, 3, 3, rotation_matrix);
	arm_mat_init_f32(&M_R_I, 3, 3, rotation_matrix_inv);
	arm_mat_inverse_f32(&M_R, &M_R_I);
}

void convert_to_hat_matrix(float* original_matrix, float* hat_matrix)
{
	hat_matrix[0] = 0;
	hat_matrix[1] = -original_matrix[2];
	hat_matrix[2] = original_matrix[1];
	hat_matrix[3] = original_matrix[2];
	hat_matrix[4] = 0;
	hat_matrix[5] = -original_matrix[0];
	hat_matrix[6] = -original_matrix[1];
	hat_matrix[7] = original_matrix[0];
	hat_matrix[8] = 0;
}

void inverse_hat_matrix(float* original_matrix, float* inverse_matrix)
{
	inverse_matrix[0] = original_matrix[7];
	inverse_matrix[1] = original_matrix[2];
	inverse_matrix[2] = original_matrix[3];
}

void Geometric_Tracking_Control(MPU9250_t* MPU9250, madgwick_t* Madgwick, camera_t* camera, float* thrust, float time_period)
{
	float32_t M_R[9], M_R_I[9], M_R_des[9], M_R_des_T[9], M_R_dot[9], M_R_des_dot[9], M_R_des_dot2[9];
	float32_t M_F_des[3], M_F_des_dot[3], M_X_B_des[3], M_X_C_des[3], M_X_C_des_dot[3], M_Y_B_des[3], M_Y_B_des_h[9], M_Z_B_des[3], M_Z_B_des_h[9];
	float32_t M_matrix_1[9], M_matrix_2[9], M_matrix_3[3], M_matrix_4[3], M_matrix_5[3], M_C[3];
	float32_t M_E_A_h[9], M_E_A_h_2[9], M_E_R[3], M_E_ang_vel[3];
	float32_t M_ang_vel[3], M_ang_vel_h[9], M_ang_vel_des_h[9], M_ang_acc_des[3];
	float32_t M_acc_ang_moment[3];

	arm_matrix_instance_f32 R, R_I, R_des, R_des_T, R_des_dot, R_des_dot2, R_dot;
	arm_matrix_instance_f32 F_des, F_des_dot, X_B_des, X_C_des, Y_B_des, Y_B_des_h, Z_B_des, Z_B_des_h;	//, R, R_I, R_des_T
	arm_matrix_instance_f32 matrix_1, matrix_2, matrix_3, matrix_4, matrix_5, C;
	arm_matrix_instance_f32 E_A_h, E_A_h_2, E_ang_vel;
	arm_matrix_instance_f32 ang_vel, ang_vel_h, ang_vel_des, ang_vel_des_dot, ang_vel_des_h, ang_acc_des;
	arm_matrix_instance_f32 acc_ang_moment, moment;

	float32_t Result, scale;

	arm_mat_init_f32(&acc_ang_moment, 3, 1, M_acc_ang_moment);
	arm_mat_init_f32(&ang_acc_des, 3, 1, M_ang_acc_des);
	arm_mat_init_f32(&ang_vel_des, 3, 1, M_ang_vel_des);
	arm_mat_init_f32(&ang_vel_des_dot, 3, 1, M_ang_vel_des_dot);
	arm_mat_init_f32(&ang_vel_des_h, 3, 3, M_ang_vel_des_h);
	arm_mat_init_f32(&ang_vel, 3, 1, M_ang_vel);
	arm_mat_init_f32(&ang_vel_h, 3, 3, M_ang_vel_h);
	arm_mat_init_f32(&C, 3, 1, M_C);
	arm_mat_init_f32(&E_A_h, 3, 3, M_E_A_h);
	arm_mat_init_f32(&E_A_h_2, 3, 3, M_E_A_h_2);
	arm_mat_init_f32(&E_ang_vel, 3, 1, M_E_ang_vel);
	arm_mat_init_f32(&F_des, 3, 1, M_F_des);
	arm_mat_init_f32(&F_des_dot, 3, 1, M_F_des_dot);
	arm_mat_init_f32(&matrix_1, 3, 3, M_matrix_1);
	arm_mat_init_f32(&matrix_2, 3, 3, M_matrix_2);
	arm_mat_init_f32(&matrix_3, 3, 1, M_matrix_3);
	arm_mat_init_f32(&matrix_4, 3, 1, M_matrix_4);
	arm_mat_init_f32(&matrix_5, 3, 1, M_matrix_5);
	arm_mat_init_f32(&moment, 3, 3, M_moment);
	arm_mat_init_f32(&R, 3, 3, M_R);
	arm_mat_init_f32(&R_I, 3, 3, M_R_I);
	arm_mat_init_f32(&R_des, 3, 3, M_R_des);
	arm_mat_init_f32(&R_des_dot, 3, 3, M_R_des_dot);
	arm_mat_init_f32(&R_des_dot2, 3, 3, M_R_des_dot2);
	arm_mat_init_f32(&R_des_T, 3, 3, M_R_des_T);
	arm_mat_init_f32(&R_dot, 3, 3, M_R_dot);
	arm_mat_init_f32(&X_C_des, 3, 1, M_X_C_des);
	arm_mat_init_f32(&X_B_des, 3, 1, M_X_B_des);
	arm_mat_init_f32(&Y_B_des, 3, 1, M_Y_B_des);
	arm_mat_init_f32(&Y_B_des_h, 3, 3, M_Y_B_des_h);
	arm_mat_init_f32(&Z_B_des, 3, 1, M_Z_B_des);
	arm_mat_init_f32(&Z_B_des_h, 3, 3, M_Z_B_des_h);

	M_ang_vel[0] = MPU9250->Gx*0.0174533f;
	M_ang_vel[1] = MPU9250->Gy*0.0174533f;
	M_ang_vel[2] = MPU9250->Gz*0.0174533f;

	//Receive from matlab
	M_F_des[0] = camera->des_force_y;
	M_F_des[1] = -camera->des_force_x;
	//M_F_des[2] = camera->des_force_z;
	M_F_des[2] = 9.80665f + camera->err_z*0.01;

	camera->Yaw_rad = camera->Yaw*0.0174533;
	M_X_C_des[0] = (arm_cos_f32(0));
	M_X_C_des[1] = (arm_sin_f32(0));
	M_X_C_des[2] = 0;

	//Calculate Rotation Matrix, input roll, pitch, yaw angle
	Rotation_Matrix(&Madgwick->Roll_rad, &Madgwick->Pitch_rad, &camera->Yaw_rad, M_R, M_R_I);		// Get rotation matrix

	//Desired body frame z-orientation
	arm_power_f32(M_F_des, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&F_des, 1.0f/scale, &Z_B_des);		// Get Z_B_des

	convert_to_hat_matrix(M_Z_B_des, M_Z_B_des_h);

	//Desired body frame y-orientation
	arm_mat_mult_f32(&Z_B_des_h, &X_C_des, &matrix_4);	// Z_B_des cross product X_C_des
	arm_power_f32(M_matrix_4, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&matrix_4, 1.0f/scale, &Y_B_des);		// Get Y_B_des

	//Desired body frame x-orientation
	convert_to_hat_matrix(M_Y_B_des, M_Y_B_des_h);
	arm_mat_mult_f32(&Y_B_des_h, &Z_B_des, &matrix_4);
	arm_power_f32(M_matrix_4, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&matrix_4, 1.0f/scale, &X_B_des);		// Get X_B_des

	//Desired attitude matrix
	for(int i=0; i<3; i++) {
		M_R_des[0+i*3] = M_X_B_des[i];
		M_R_des[1+i*3] = M_Y_B_des[i];
		M_R_des[2+i*3] = M_Z_B_des[i];
	}

	//Calculate Error
	arm_mat_trans_f32(&R_des, &R_des_T);
	arm_mat_mult_f32(&R_des_T, &R, &matrix_1);
	arm_mat_mult_f32(&R_I, &R_des, &matrix_2);
	arm_mat_sub_f32(&matrix_1, &matrix_2, &E_A_h);
	arm_mat_scale_f32(&E_A_h, 0.5, &E_A_h_2);
	inverse_hat_matrix(M_E_A_h_2, M_E_R);		// Get error orientation M_E_R

	//Get R_des_dot
	arm_sub_f32(M_R_des, last_M_R_des, M_matrix_1, 9);
	arm_mat_scale_f32(&matrix_1, (1/time_period), &R_des_dot);
	arm_copy_f32(M_R_des, last_M_R_des, 9);

	//Get angular velocity hat matrix
	arm_mat_mult_f32(&R_des_T, &R_des_dot, &matrix_1);
	inverse_hat_matrix(M_matrix_1, M_ang_vel_des);

	// Get angular_velocity_des_dot
	arm_sub_f32(M_ang_vel_des, last_M_ang_vel_des, M_matrix_4, 3);
	arm_mat_scale_f32(&matrix_4, (1/time_period), &ang_vel_des_dot);
	arm_copy_f32(M_ang_vel_des, last_M_ang_vel_des, 3);

	//Get error angular velocity
	arm_mat_mult_f32(&R_I, &R_des, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &ang_vel_des, &matrix_4);
	convert_to_hat_matrix(M_matrix_4, M_matrix_1);
	arm_mat_sub_f32(&ang_vel, &matrix_4, &E_ang_vel);

	//Calculate angular acceleration error
	arm_mat_mult_f32(&matrix_1, &moment, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &R_I, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &R_des, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &ang_vel_des, &matrix_3);

	arm_mat_mult_f32(&moment, &R_I, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &R_des, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &ang_vel_des_dot, &matrix_4);
	arm_mat_add_f32(&matrix_3, &matrix_4, &acc_ang_moment);

	//Get accum error angular velocity
	for(int i=0; i<3; i++) {
		const float32_t c = 2, d = 1, limit = 6;
		M_accum_attitude_error[i] += (d*M_E_ang_vel[i] + c*M_E_R[i])*time_period;
		if(M_accum_attitude_error[i]>limit)M_accum_attitude_error[i] = limit;
		else if(M_accum_attitude_error[i]<-limit)M_accum_attitude_error[i] = -limit;
	}

	thrust[1] = -Kr1*M_E_R[0] - Kg1*M_E_ang_vel[0] + Km*M_acc_ang_moment[0] - Ki1*M_accum_attitude_error[0];
	thrust[2] = -Kr2*M_E_R[1] - Kg2*M_E_ang_vel[1] + Km*M_acc_ang_moment[1] - Ki2*M_accum_attitude_error[1];
	thrust[3] = -Kr3*M_E_R[2] - Kg3*M_E_ang_vel[2] + Km*M_acc_ang_moment[2] - Ki3*M_accum_attitude_error[2];
}

void Geometric_Tracking_Control_Manual(MPU9250_t* MPU9250, madgwick_t* Madgwick, camera_t* camera, futaba_t* futaba, float* thrust, float time_period)
{
	float32_t M_R[9], M_R_I[9], M_R_des[9], M_R_des_T[9], M_R_dot[9], M_R_des_dot[9], M_R_des_dot2[9];
	float32_t M_F_des[3], M_F_des_dot[3], M_X_B_des[3], M_X_C_des[3], M_Y_B_des[3], M_Y_B_des_h[9], M_Z_B_des[3], M_Z_B_des_h[9];
	float32_t M_matrix_1[9], M_matrix_2[9], M_matrix_3[3], M_matrix_4[3], M_matrix_5[3], M_C[3];
	float32_t M_E_A_h[9], M_E_A_h_2[9], M_E_R[3], M_E_ang_vel[3];
	float32_t M_ang_vel[3], M_ang_vel_h[9], M_ang_vel_des_h[9], M_ang_acc_des[3];
	float32_t M_acc_ang_moment[3];

	arm_matrix_instance_f32 R, R_I, R_des, R_des_T, R_des_dot, R_des_dot2, R_dot;
	arm_matrix_instance_f32 F_des, F_des_dot, X_B_des, X_C_des, Y_B_des, Y_B_des_h, Z_B_des, Z_B_des_h;	//, R, R_I, R_des_T
	arm_matrix_instance_f32 matrix_1, matrix_2, matrix_3, matrix_4, matrix_5, C;
	arm_matrix_instance_f32 E_A_h, E_A_h_2, E_ang_vel;
	arm_matrix_instance_f32 ang_vel, ang_vel_h, ang_vel_des, ang_vel_des_dot, ang_vel_des_h, ang_acc_des;
	arm_matrix_instance_f32 acc_ang_moment, moment;

	float32_t Result, scale;

	arm_mat_init_f32(&acc_ang_moment, 3, 1, M_acc_ang_moment);
	arm_mat_init_f32(&ang_acc_des, 3, 1, M_ang_acc_des);
	arm_mat_init_f32(&ang_vel_des, 3, 1, M_ang_vel_des);
	arm_mat_init_f32(&ang_vel_des_dot, 3, 1, M_ang_vel_des_dot);
	arm_mat_init_f32(&ang_vel_des_h, 3, 3, M_ang_vel_des_h);
	arm_mat_init_f32(&ang_vel, 3, 1, M_ang_vel);
	arm_mat_init_f32(&ang_vel_h, 3, 3, M_ang_vel_h);
	arm_mat_init_f32(&C, 3, 1, M_C);
	arm_mat_init_f32(&E_A_h, 3, 3, M_E_A_h);
	arm_mat_init_f32(&E_A_h_2, 3, 3, M_E_A_h_2);
	arm_mat_init_f32(&E_ang_vel, 3, 1, M_E_ang_vel);
	arm_mat_init_f32(&F_des, 3, 1, M_F_des);
	arm_mat_init_f32(&F_des_dot, 3, 1, M_F_des_dot);
	arm_mat_init_f32(&matrix_1, 3, 3, M_matrix_1);
	arm_mat_init_f32(&matrix_2, 3, 3, M_matrix_2);
	arm_mat_init_f32(&matrix_3, 3, 1, M_matrix_3);
	arm_mat_init_f32(&matrix_4, 3, 1, M_matrix_4);
	arm_mat_init_f32(&matrix_5, 3, 1, M_matrix_5);
	arm_mat_init_f32(&moment, 3, 3, M_moment);
	arm_mat_init_f32(&R, 3, 3, M_R);
	arm_mat_init_f32(&R_I, 3, 3, M_R_I);
	arm_mat_init_f32(&R_des, 3, 3, M_R_des);
	arm_mat_init_f32(&R_des_dot, 3, 3, M_R_des_dot);
	arm_mat_init_f32(&R_des_dot2, 3, 3, M_R_des_dot2);
	arm_mat_init_f32(&R_des_T, 3, 3, M_R_des_T);
	arm_mat_init_f32(&R_dot, 3, 3, M_R_dot);
	arm_mat_init_f32(&X_C_des, 3, 1, M_X_C_des);
	arm_mat_init_f32(&X_B_des, 3, 1, M_X_B_des);
	arm_mat_init_f32(&Y_B_des, 3, 1, M_Y_B_des);
	arm_mat_init_f32(&Y_B_des_h, 3, 3, M_Y_B_des_h);
	arm_mat_init_f32(&Z_B_des, 3, 1, M_Z_B_des);
	arm_mat_init_f32(&Z_B_des_h, 3, 3, M_Z_B_des_h);

	M_ang_vel[0] = MPU9250->Gx*0.0174533f;
	M_ang_vel[1] = MPU9250->Gy*0.0174533f;
	M_ang_vel[2] = MPU9250->Gz*0.0174533f;

	float32_t cmd_roll = futaba->roll - roll_middle;
	if((cmd_roll) > 6 || (cmd_roll) < -6) {
		M_F_des[1] = -cmd_roll*0.015;
	} else M_F_des[1] = 0;

	float32_t cmd_pitch = futaba->pitch - pitch_middle;
	if((cmd_pitch) > 6 || (cmd_pitch) < -6) {
		M_F_des[0] = -cmd_pitch*0.015;
	} else M_F_des[0] = 0;

	M_F_des[2] = 9.80665f;

	float32_t cmd_yaw = futaba->yaw - yaw_middle;
	if((cmd_yaw) > 7 || (cmd_yaw) < -7) {
		Yaw -= cmd_yaw*0.0005f*0.0174533f;
	} else Yaw += 0;

	M_X_C_des[0] = (arm_cos_f32(Yaw));
	M_X_C_des[1] = (arm_sin_f32(Yaw));
	M_X_C_des[2] = 0;

	//Calculate Rotation Matrix, input roll, pitch, yaw angle
	camera->Yaw_rad = camera->Yaw*0.0174533;
	Rotation_Matrix(&Madgwick->Roll_rad, &Madgwick->Pitch_rad, &camera->Yaw_rad, M_R, M_R_I); //Get rotation matrix

	//Control Z throttle
	thrust[0] = (futaba->throttle - throttle_min)*1.64;

	//Desired body frame z-orientation/
	arm_power_f32(M_F_des, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&F_des, 1.0f/scale, &Z_B_des); //Get Z_B_des

	convert_to_hat_matrix(M_Z_B_des, M_Z_B_des_h);

	//Desired body frame y-orientation
	arm_mat_mult_f32(&Z_B_des_h, &X_C_des, &matrix_4); //Z_B_des cross product X_C_des
	arm_power_f32(M_matrix_4, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&matrix_4, 1.0f/scale, &Y_B_des); // Get Y_B_des

	//Desired body frame x-orientation
	convert_to_hat_matrix(M_Y_B_des, M_Y_B_des_h);
	arm_mat_mult_f32(&Y_B_des_h, &Z_B_des, &matrix_4);
	arm_power_f32(M_matrix_4, 3, &Result);
	scale = sqrt(Result);
	arm_mat_scale_f32(&matrix_4, 1.0f/scale, &X_B_des); //Get X_B_des

	//Desired attitude matrix
	for(int i=0; i<3; i++) {
		M_R_des[0+i*3] = M_X_B_des[i];
		M_R_des[1+i*3] = M_Y_B_des[i];
		M_R_des[2+i*3] = M_Z_B_des[i];
	}

	//Calculate Error
	arm_mat_trans_f32(&R_des, &R_des_T);
	arm_mat_mult_f32(&R_des_T, &R, &matrix_1);
	arm_mat_mult_f32(&R_I, &R_des, &matrix_2);
	arm_mat_sub_f32(&matrix_1, &matrix_2, &E_A_h);
	arm_mat_scale_f32(&E_A_h, 0.5, &E_A_h_2);
	inverse_hat_matrix(M_E_A_h_2, M_E_R);		// Get error orientation M_E_R

	//Get R_des_dot
	arm_sub_f32(M_R_des, last_M_R_des, M_matrix_1, 9);
	arm_mat_scale_f32(&matrix_1, (1/time_period), &R_des_dot);
	arm_copy_f32(M_R_des, last_M_R_des, 9);

	//Get angular velocity hat matrix
	arm_mat_mult_f32(&R_des_T, &R_des_dot, &matrix_1);
	inverse_hat_matrix(M_matrix_1, M_ang_vel_des);

	//Get angular_velocity_des_dot
	arm_sub_f32(M_ang_vel_des, last_M_ang_vel_des, M_matrix_4, 3);
	arm_mat_scale_f32(&matrix_4, (1/time_period), &ang_vel_des_dot);
	arm_copy_f32(M_ang_vel_des, last_M_ang_vel_des, 3);

	//Get error angular velocity
	arm_mat_mult_f32(&R_I, &R_des, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &ang_vel_des, &matrix_4);
	convert_to_hat_matrix(M_matrix_4, M_matrix_1);
	arm_mat_sub_f32(&ang_vel, &matrix_4, &E_ang_vel);

	//Calculate angular acceleration error
	arm_mat_mult_f32(&matrix_1, &moment, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &R_I, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &R_des, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &ang_vel_des, &matrix_3);

	arm_mat_mult_f32(&moment, &R_I, &matrix_1);
	arm_mat_mult_f32(&matrix_1, &R_des, &matrix_2);
	arm_mat_mult_f32(&matrix_2, &ang_vel_des_dot, &matrix_4);
	arm_mat_add_f32(&matrix_3, &matrix_4, &acc_ang_moment);

	//Get accum error angular velocity
	for(int i=0; i<3; i++) {
		const float32_t c = 2, d = 1, limit = 6;
		M_accum_attitude_error[i] += (d*M_E_ang_vel[i] + c*M_E_R[i])*time_period;
		if(M_accum_attitude_error[i]>limit)M_accum_attitude_error[i] = limit;
		else if(M_accum_attitude_error[i]<-limit)M_accum_attitude_error[i] = -limit;
	}

	thrust[1] = -Kr1*M_E_R[0] - Kg1*M_E_ang_vel[0] + Km*M_acc_ang_moment[0] - Ki1*M_accum_attitude_error[0];
	thrust[2] = -Kr2*M_E_R[1] - Kg2*M_E_ang_vel[1] + Km*M_acc_ang_moment[1] - Ki2*M_accum_attitude_error[1];
	thrust[3] = -Kr3*M_E_R[2] - Kg3*M_E_ang_vel[2] + Km*M_acc_ang_moment[2] - Ki3*M_accum_attitude_error[2];
}
