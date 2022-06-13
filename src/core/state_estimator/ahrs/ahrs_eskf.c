#include <math.h>
#include "arm_math.h"
#include "ahrs.h"
#include "lpf.h"
#include "matrix.h"
#include "se3_math.h"
#include "quaternion.h"
#include "ahrs_comp_filter.h"
#include "compass.h"
#include "imu.h"

MAT_ALLOC(x_nominal, 4, 1);     //x = [q0; q1; q2; q3]
MAT_ALLOC(x_error_state, 3, 1); //delta_x = [theta_x; theta_y; theta_z]
MAT_ALLOC(F_x, 3, 3);
MAT_ALLOC(Q_i, 3, 3);
MAT_ALLOC(P_prior, 3, 3);
MAT_ALLOC(P_post, 3, 3);
MAT_ALLOC(V_accel, 3, 3);
MAT_ALLOC(V_mag, 3, 3);
MAT_ALLOC(F_x_t, 3, 3);
MAT_ALLOC(FP, 3, 3);
MAT_ALLOC(FPFt, 3, 3);
MAT_ALLOC(FQFt, 3, 3);
MAT_ALLOC(Q_delta_theta, 4, 3);
MAT_ALLOC(X_delta_x, 4, 3);
MAT_ALLOC(H_x_accel, 3, 4);
MAT_ALLOC(H_accel, 3, 3);
MAT_ALLOC(H_accel_t, 3, 3);
MAT_ALLOC(PHt_accel, 3, 3);
MAT_ALLOC(HPHt_accel, 3, 3);
MAT_ALLOC(HPHt_V_accel, 3, 3);
MAT_ALLOC(HPHt_V_accel_inv, 3, 3);
MAT_ALLOC(K_accel, 3, 3);
MAT_ALLOC(KH_accel, 3, 3);
MAT_ALLOC(I_KH_accel, 3, 3);
MAT_ALLOC(y_accel, 3, 1);
MAT_ALLOC(h_accel, 3, 1);
MAT_ALLOC(accel_resid, 3, 1);
MAT_ALLOC(H_x_mag, 3, 4);
MAT_ALLOC(H_mag, 3, 3);
MAT_ALLOC(H_mag_t, 3, 3);
MAT_ALLOC(PHt_mag, 3, 3);
MAT_ALLOC(HPHt_mag, 3, 3);
MAT_ALLOC(HPHt_V_mag, 3, 3);
MAT_ALLOC(HPHt_V_mag_inv, 3, 3);
MAT_ALLOC(K_mag, 3, 3);
MAT_ALLOC(KH_mag, 3, 3);
MAT_ALLOC(I_KH_mag, 3, 3);
MAT_ALLOC(y_mag, 3, 1);
MAT_ALLOC(h_mag, 3, 1);
MAT_ALLOC(mag_resid, 3, 1);

float eskf_dt;
float eskf_half_dt;

void eskf_ahrs_init(float dt)
{
	eskf_dt = dt;
	eskf_half_dt = 0.5 * dt;

	MAT_INIT(x_nominal, 4, 1);
	MAT_INIT(x_error_state, 3, 1);
	MAT_INIT(F_x, 3, 3);
	MAT_INIT(Q_i, 3, 3);
	MAT_INIT(P_prior, 3, 3);
	MAT_INIT(P_post, 3, 3);
	MAT_INIT(V_accel, 3, 3);
	MAT_INIT(V_mag, 3, 3);
	MAT_INIT(F_x_t, 3, 3);
	MAT_INIT(FP, 3, 3);
	MAT_INIT(FPFt, 3, 3);
	MAT_INIT(FQFt, 3, 3);
	MAT_INIT(Q_delta_theta, 4, 3);
	MAT_INIT(X_delta_x, 4, 3);
	MAT_INIT(H_x_accel, 3, 4);
	MAT_INIT(H_accel, 3, 3);
	MAT_INIT(H_accel_t, 3, 3);
	MAT_INIT(PHt_accel, 3, 3);
	MAT_INIT(HPHt_accel, 3, 3);
	MAT_INIT(HPHt_V_accel, 3, 3);
	MAT_INIT(HPHt_V_accel_inv, 3, 3);
	MAT_INIT(K_accel, 3, 3);
	MAT_INIT(KH_accel, 3, 3);
	MAT_INIT(I_KH_accel, 3, 3);
	MAT_INIT(y_accel, 3, 1);
	MAT_INIT(h_accel, 3, 1);
	MAT_INIT(accel_resid, 3, 1);
	MAT_INIT(H_x_mag, 3, 4);
	MAT_INIT(H_mag, 3, 3);
	MAT_INIT(H_mag_t, 3, 3);
	MAT_INIT(PHt_mag, 3, 3);
	MAT_INIT(HPHt_mag, 3, 3);
	MAT_INIT(HPHt_V_mag, 3, 3);
	MAT_INIT(HPHt_V_mag_inv, 3, 3);
	MAT_INIT(K_mag, 3, 3);
	MAT_INIT(KH_mag, 3, 3);
	MAT_INIT(I_KH_mag, 3, 3);
	MAT_INIT(y_mag, 3, 1);
	MAT_INIT(h_mag, 3, 1);
	MAT_INIT(mag_resid, 3, 1);

	/* initialize the nominal state */
	mat_data(x_nominal)[0] = 1.0f;
	mat_data(x_nominal)[1] = 0.0f;
	mat_data(x_nominal)[2] = 0.0f;
	mat_data(x_nominal)[3] = 0.0f;

	/* initialize Q_i matrix */
	mat_data(Q_i)[0*3 + 0] = 1e-5;
	mat_data(Q_i)[0*3 + 1] = 0.0f;
	mat_data(Q_i)[0*3 + 2] = 0.0f;

	mat_data(Q_i)[1*3 + 0] = 0.0f;
	mat_data(Q_i)[1*3 + 1] = 1e-5;
	mat_data(Q_i)[1*3 + 2] = 0.0f;

	mat_data(Q_i)[2*3 + 0] = 0.0f;
	mat_data(Q_i)[2*3 + 1] = 0.0f;
	mat_data(Q_i)[2*3 + 2] = 1e-5;

	/* initialize P matrix */
	mat_data(P_post)[0*3 + 0] = 5.0f;
	mat_data(P_post)[0*3 + 1] = 0.0f;
	mat_data(P_post)[0*3 + 2] = 0.0f;

	mat_data(P_post)[1*3 + 0] = 0.0f;
	mat_data(P_post)[1*3 + 1] = 5.0f;
	mat_data(P_post)[1*3 + 2] = 0.0f;

	mat_data(P_post)[2*3 + 0] = 0.0f;
	mat_data(P_post)[2*3 + 1] = 0.0f;
	mat_data(P_post)[2*3 + 2] = 5.0f;

	/* initialize V_accel matrix */
	mat_data(V_accel)[0*3 + 0] = 7e-1;
	mat_data(V_accel)[0*3 + 1] = 0.0f;
	mat_data(V_accel)[0*3 + 2] = 0.0f;

	mat_data(V_accel)[1*3 + 0] = 0.0f;
	mat_data(V_accel)[1*3 + 1] = 7e-1;
	mat_data(V_accel)[1*3 + 2] = 0.0f;

	mat_data(V_accel)[2*3 + 0] = 0.0f;
	mat_data(V_accel)[2*3 + 1] = 0.0f;
	mat_data(V_accel)[2*3 + 2] = 7e-1;

	/* initialize V_mag matrix */
	mat_data(V_mag)[0*3 + 0] = 5e-1;
	mat_data(V_mag)[0*3 + 1] = 0.0f;
	mat_data(V_mag)[0*3 + 2] = 0.0f;

	mat_data(V_mag)[1*3 + 0] = 0.0f;
	mat_data(V_mag)[1*3 + 1] = 5e-1;
	mat_data(V_mag)[1*3 + 2] = 0.0f;

	mat_data(V_mag)[2*3 + 0] = 0.0f;
	mat_data(V_mag)[2*3 + 1] = 0.0f;
	mat_data(V_mag)[2*3 + 2] = 5e-1;

	int r, c;
	for(r = 0; r < 3; r++) {
		for(c = 0; c < 3; c++) {
			mat_data(FQFt)[r*3 + c] = mat_data(Q_i)[r*3 + c];
		}
	}

	init_ahrs_quaternion_with_accel_and_compass(mat_data(x_nominal));
	mat_data(x_nominal)[1] *= -1;
	mat_data(x_nominal)[2] *= -1;
	mat_data(x_nominal)[3] *= -1;
}

void set_ahrs_eskf_quaternion(float *q_new)
{
	quaternion_copy(&mat_data(x_nominal)[0], q_new);
}

void eskf_ahrs_predict(float *gyro)
{
	/* update nominal state (quaternion integration) */
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[0];
	w[2] = gyro[1];
	w[3] = gyro[2];

	float q_dot[4];
	quaternion_mult(mat_data(x_nominal), w, q_dot);

	mat_data(x_nominal)[0] = mat_data(x_nominal)[0] + (q_dot[0] * eskf_half_dt);
	mat_data(x_nominal)[1] = mat_data(x_nominal)[1] + (q_dot[1] * eskf_half_dt);
	mat_data(x_nominal)[2] = mat_data(x_nominal)[2] + (q_dot[2] * eskf_half_dt);
	mat_data(x_nominal)[3] = mat_data(x_nominal)[3] + (q_dot[3] * eskf_half_dt);
	quat_normalize(mat_data(x_nominal));

	/* construct error state transition matrix */
	mat_data(F_x)[0*3 + 0] = eskf_dt;
	mat_data(F_x)[0*3 + 1] = +gyro[2] * eskf_dt;
	mat_data(F_x)[0*3 + 2] = -gyro[1] * eskf_dt;

	mat_data(F_x)[1*3 + 0] = -gyro[2] * eskf_dt;
	mat_data(F_x)[1*3 + 1] = eskf_dt;
	mat_data(F_x)[1*3 + 2] = +gyro[0];

	mat_data(F_x)[2*3 + 0] = +gyro[1] * eskf_dt;
	mat_data(F_x)[2*3 + 1] = -gyro[0] * eskf_dt;
	mat_data(F_x)[2*3 + 2] = eskf_dt;

	/* update process noise covariance matrix */
	MAT_TRANS(&F_x, &F_x_t)
	MAT_MULT(&F_x, &P_post, &FP);
	MAT_MULT(&FP, &F_x_t, &FPFt);
	MAT_ADD(&FPFt, &FQFt, &P_prior);
}

void eskf_ahrs_accelerometer_correct(float *accel)
{
	mat_data(y_accel)[0] = accel[0];
	mat_data(y_accel)[1] = accel[1];
	mat_data(y_accel)[2] = accel[2];

	float q0 = mat_data(x_nominal)[0];
	float q1 = mat_data(x_nominal)[1];
	float q2 = mat_data(x_nominal)[2];
	float q3 = mat_data(x_nominal)[3];

	/* construct error state observation matrix */
	mat_data(H_x_accel)[0*4 + 0] = -2*q2;
	mat_data(H_x_accel)[0*4 + 1] = +2*q3;
	mat_data(H_x_accel)[0*4 + 2] = -2*q0;
	mat_data(H_x_accel)[0*4 + 3] = +2*q1;

	mat_data(H_x_accel)[1*4 + 0] = +2*q1;
	mat_data(H_x_accel)[1*4 + 1] = +2*q0;
	mat_data(H_x_accel)[1*4 + 2] = +2*q3;
	mat_data(H_x_accel)[1*4 + 3] = +2*q2;

	mat_data(H_x_accel)[2*4 + 0] = +2*q0;
	mat_data(H_x_accel)[2*4 + 1] = -2*q1;
	mat_data(H_x_accel)[2*4 + 2] = -2*q2;
	mat_data(H_x_accel)[2*4 + 3] = +2*q3;

	mat_data(Q_delta_theta)[0*3 + 0] = -q1;
	mat_data(Q_delta_theta)[0*3 + 1] = -q2;
	mat_data(Q_delta_theta)[0*3 + 2] = -q3;

	mat_data(Q_delta_theta)[1*3 + 0] = +q0;
	mat_data(Q_delta_theta)[1*3 + 1] = -q3;
	mat_data(Q_delta_theta)[1*3 + 2] = +q2;

	mat_data(Q_delta_theta)[2*3 + 0] = +q3;
	mat_data(Q_delta_theta)[2*3 + 1] = +q0;
	mat_data(Q_delta_theta)[2*3 + 2] = -q1;

	mat_data(Q_delta_theta)[3*3 + 0] = -q2;
	mat_data(Q_delta_theta)[3*3 + 1] = +q1;
	mat_data(Q_delta_theta)[3*3 + 2] = +q0;

	MAT_SCALE(&Q_delta_theta, 0.5, &X_delta_x);
	MAT_MULT(&H_x_accel, &X_delta_x, &H_accel)

	/* use the state from prediction step (gyroscope) to predict the
	 * gravity vector */
	mat_data(h_accel)[0] = 2 * (-q0*q2 + q1*q3);
	mat_data(h_accel)[1] = 2 * (q2*q3 + q0*q1);
	mat_data(h_accel)[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_TRANS(&H_accel, &H_accel_t);
	MAT_MULT(&P_prior, &H_accel_t, &PHt_accel);
	MAT_MULT(&H_accel, &PHt_accel, &HPHt_accel);
	MAT_ADD(&HPHt_accel, &V_accel, &HPHt_V_accel);
	MAT_INV(&HPHt_V_accel, &HPHt_V_accel_inv);
	MAT_MULT(&PHt_accel, &HPHt_V_accel_inv, &K_accel);

	/* calculate error state residual */
	//delta_x = K * (y_accel - h_accel)
	MAT_SUB(&y_accel, &h_accel, &accel_resid);
	MAT_MULT(&K_accel, &accel_resid, &x_error_state);

	/* calculate a posteriori process covariance matrix */
	//P = (I - K*H) * P
	MAT_MULT(&K_accel, &H_accel, &KH_accel);
	mat_data(I_KH_accel)[0*3 + 0] = 1.0f - mat_data(KH_accel)[0*3 + 0];
	mat_data(I_KH_accel)[0*3 + 1] =        mat_data(KH_accel)[0*3 + 1];
	mat_data(I_KH_accel)[0*3 + 2] =        mat_data(KH_accel)[0*3 + 2];

	mat_data(I_KH_accel)[1*3 + 0] =        mat_data(KH_accel)[1*3 + 0];
	mat_data(I_KH_accel)[1*3 + 1] = 1.0f - mat_data(KH_accel)[1*3 + 1];
	mat_data(I_KH_accel)[1*3 + 2] =        mat_data(KH_accel)[1*3 + 2];

	mat_data(I_KH_accel)[2*3 + 0] =        mat_data(KH_accel)[2*3 + 0];
	mat_data(I_KH_accel)[2*3 + 1] =        mat_data(KH_accel)[2*3 + 1];
	mat_data(I_KH_accel)[2*3 + 2] = 1.0f - mat_data(KH_accel)[2*3 + 2];
	MAT_MULT(&I_KH_accel, &P_prior, &P_post);

	/* P_post becoms the P_prior of the magnatometer correction */
	memcpy(mat_data(P_prior), mat_data(P_post), sizeof(float) * 9);

	/* error state injection */
	float q_error[4];
	q_error[0] = 1.0f;
	q_error[1] = 0.5 * mat_data(x_error_state)[0];
	q_error[2] = 0.5 * mat_data(x_error_state)[1];
	q_error[3] = 0.5 * mat_data(x_error_state)[2];

	//x_nominal (a posteriori) = q_error * x_nominal (a priori)
	float x_last[4];
	quaternion_copy(x_last, mat_data(x_nominal));
	quaternion_mult(x_last, q_error, mat_data(x_nominal));

	//renormailization
	quat_normalize(mat_data(x_nominal));
}

void eskf_ahrs_magnetometer_correct(float *mag)
{
	/* normalization */
	normalize_3x1(mag);

	mat_data(y_mag)[0] = mag[0];
	mat_data(y_mag)[1] = mag[1];
	mat_data(y_mag)[2] = mag[2];

	float q0 = mat_data(x_nominal)[0];
	float q1 = mat_data(x_nominal)[1];
	float q2 = mat_data(x_nominal)[2];
	float q3 = mat_data(x_nominal)[3];

	float gamma = sqrt(mag[0]*mag[0] + mag[1]*mag[1]);

	/* construct error state observation matrix */
	mat_data(H_x_mag)[0*4 + 0] = 2*(+gamma*q0 - mag[2]*q2);
	mat_data(H_x_mag)[0*4 + 1] = 2*(+gamma*q1 + mag[2]*q3);
	mat_data(H_x_mag)[0*4 + 2] = 2*(-gamma*q2 - mag[2]*q0);
	mat_data(H_x_mag)[0*4 + 3] = 2*(-gamma*q3 + mag[2]*q1);

	mat_data(H_x_mag)[1*4 + 0] = 2*(-gamma*q3 + mag[2]*q1);
	mat_data(H_x_mag)[1*4 + 1] = 2*(+gamma*q2 + mag[2]*q0);
	mat_data(H_x_mag)[1*4 + 2] = 2*(+gamma*q1 + mag[2]*q3);
	mat_data(H_x_mag)[1*4 + 3] = 2*(-gamma*q0 + mag[2]*q2);

	mat_data(H_x_mag)[2*4 + 0] = 2*(+gamma*q2 + mag[2]*q0);
	mat_data(H_x_mag)[2*4 + 1] = 2*(+gamma*q3 - mag[2]*q1);
	mat_data(H_x_mag)[2*4 + 2] = 2*(+gamma*q0 - mag[2]*q2);
	mat_data(H_x_mag)[2*4 + 3] = 2*(+gamma*q1 + mag[2]*q3);

	mat_data(Q_delta_theta)[0*3 + 0] = -q1;
	mat_data(Q_delta_theta)[0*3 + 1] = -q2;
	mat_data(Q_delta_theta)[0*3 + 2] = -q3;

	mat_data(Q_delta_theta)[1*3 + 0] = +q0;
	mat_data(Q_delta_theta)[1*3 + 1] = -q3;
	mat_data(Q_delta_theta)[1*3 + 2] = +q2;

	mat_data(Q_delta_theta)[2*3 + 0] = +q3;
	mat_data(Q_delta_theta)[2*3 + 1] = +q0;
	mat_data(Q_delta_theta)[2*3 + 2] = -q1;

	mat_data(Q_delta_theta)[3*3 + 0] = -q2;
	mat_data(Q_delta_theta)[3*3 + 1] = +q1;
	mat_data(Q_delta_theta)[3*3 + 2] = +q0;

	MAT_SCALE(&Q_delta_theta, 0.5, &X_delta_x);
	MAT_MULT(&H_x_mag, &X_delta_x, &H_mag)

	/* use the state from prediction step (gyroscope) and stage1 correction (accelerometer)
	 * to predict the magnetic field vector */
	mat_data(h_mag)[0] = gamma*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2*mag[2]*(q1*q3 - q0*q2);
	mat_data(h_mag)[1] = 2*gamma*(q1*q2 - q0*q3) + 2*mag[2]*(q2*q3 + q0*q1);
	mat_data(h_mag)[2] = 2*gamma*(q1*q3 + q0*q2) + mag[2]*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_TRANS(&H_mag, &H_mag_t);
	MAT_MULT(&P_prior, &H_mag_t, &PHt_mag);
	MAT_MULT(&H_mag, &PHt_mag, &HPHt_mag);
	MAT_ADD(&HPHt_mag, &V_mag, &HPHt_V_mag);
	MAT_INV(&HPHt_V_mag, &HPHt_V_mag_inv);
	MAT_MULT(&PHt_mag, &HPHt_V_mag_inv, &K_mag);

	/* calculate error state residual */
	//delta_x = K * (y_mag - h_mag)
	MAT_SUB(&y_mag, &h_mag, &mag_resid);
	MAT_MULT(&K_mag, &mag_resid, &x_error_state);

	/* calculate a posteriori process covariance matrix */
	//P = (I - K*H) * P
	MAT_MULT(&K_mag, &H_mag, &KH_mag);
	mat_data(I_KH_mag)[0*3 + 0] = 1.0f - mat_data(KH_mag)[0*3 + 0];
	mat_data(I_KH_mag)[0*3 + 1] =        mat_data(KH_mag)[0*3 + 1];
	mat_data(I_KH_mag)[0*3 + 2] =        mat_data(KH_mag)[0*3 + 2];

	mat_data(I_KH_mag)[1*3 + 0] =        mat_data(KH_mag)[1*3 + 0];
	mat_data(I_KH_mag)[1*3 + 1] = 1.0f - mat_data(KH_mag)[1*3 + 1];
	mat_data(I_KH_mag)[1*3 + 2] =        mat_data(KH_mag)[1*3 + 2];

	mat_data(I_KH_mag)[2*3 + 0] =        mat_data(KH_mag)[2*3 + 0];
	mat_data(I_KH_mag)[2*3 + 1] =        mat_data(KH_mag)[2*3 + 1];
	mat_data(I_KH_mag)[2*3 + 2] = 1.0f - mat_data(KH_mag)[2*3 + 2];
	MAT_MULT(&I_KH_mag, &P_prior, &P_post);

	/* error state injection */
	float q_error[4];
	q_error[0] = 1.0f;
	q_error[1] = 0.0f; //0.5 * mat_data(x_error_state)[0];
	q_error[2] = 0.0f; //0.5 * mat_data(x_error_state)[1];
	q_error[3] = 0.5 * mat_data(x_error_state)[2];

	//x_nominal (a posteriori) = q_error * x_nominal (a priori)
	float x_last[4];
	quaternion_copy(x_last, mat_data(x_nominal));
	quaternion_mult(x_last, q_error, mat_data(x_nominal));

	//renormailization
	quat_normalize(mat_data(x_nominal));
}

void get_eskf_attitude_quaternion(float *q_out)
{
	q_out[0] = mat_data(x_nominal)[0];
	q_out[1] = mat_data(x_nominal)[1];
	q_out[2] = mat_data(x_nominal)[2];
	q_out[3] = mat_data(x_nominal)[3];
}
