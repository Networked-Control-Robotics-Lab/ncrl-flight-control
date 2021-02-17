#include <math.h>
#include "arm_math.h"
#include "ahrs.h"
#include "lpf.h"
#include "matrix.h"
#include "se3_math.h"
#include "quaternion.h"

/*==================================================================*
 * error-state kalman filter for multirotor full state estimation   *
 * derived by Sheng-Wen, Cheng (shengwen1997.tw@gmail.com)          *
 * estimate state = [position(3x1); velocity(3x1); quaternion(4x1)] *
 *==================================================================*/

#define ESKF_RESCALE(number) (number * 10e7) //to improve the numerical stability

MAT_ALLOC(x_nominal, 10, 1);
MAT_ALLOC(x_error_state, 9, 1);
MAT_ALLOC(F_x, 9, 9);
MAT_ALLOC(Q_i, 6, 6);
MAT_ALLOC(P_prior, 9, 9);
MAT_ALLOC(P_post, 9, 9);
MAT_ALLOC(V_accel, 3, 3);
MAT_ALLOC(V_mag, 3, 3);
MAT_ALLOC(F_x_t, 9, 9);
MAT_ALLOC(FP, 9, 9);
MAT_ALLOC(FPFt, 9, 9);
MAT_ALLOC(FQFt, 9, 9);
MAT_ALLOC(Q_delta_theta, 4, 3);
MAT_ALLOC(X_delta_x, 10, 9);
MAT_ALLOC(H_x_accel, 3, 10);
MAT_ALLOC(H_accel, 3, 9);
MAT_ALLOC(H_accel_t, 9, 3);
MAT_ALLOC(PHt_accel, 9, 3);
MAT_ALLOC(HPHt_accel, 3, 3);
MAT_ALLOC(HPHt_V_accel, 3, 3);
MAT_ALLOC(HPHt_V_accel_inv, 3, 3);
MAT_ALLOC(K_accel, 9, 3);
MAT_ALLOC(KH_accel, 9, 9);
MAT_ALLOC(I_KH_accel, 9, 9);
MAT_ALLOC(y_accel, 3, 1);
MAT_ALLOC(h_accel, 3, 1);
MAT_ALLOC(accel_resid, 3, 1);
MAT_ALLOC(H_x_mag, 3, 10);
MAT_ALLOC(H_mag, 3, 9);
MAT_ALLOC(H_mag_t, 9, 3);
MAT_ALLOC(PHt_mag, 9, 3);
MAT_ALLOC(HPHt_mag, 3, 3);
MAT_ALLOC(HPHt_V_mag, 3, 3);
MAT_ALLOC(HPHt_V_mag_inv, 3, 3);
MAT_ALLOC(K_mag, 9, 3);
MAT_ALLOC(KH_mag, 9, 9);
MAT_ALLOC(I_KH_mag, 9, 9);
MAT_ALLOC(y_mag, 3, 1);
MAT_ALLOC(h_mag, 3, 1);
MAT_ALLOC(mag_resid, 3, 1);

float dt;
float neg_half_dt;
float half_dt_squared;

void eskf_ins_init(float dt)
{
	dt = dt;
	neg_half_dt = -0.5f * dt;
	half_dt_squared = 0.5f * dt * dt;

	MAT_INIT(x_nominal, 10, 1);
	MAT_INIT(x_error_state, 9, 1);
	MAT_INIT(F_x, 9, 9);
	MAT_INIT(Q_i, 6, 6);
	MAT_INIT(P_prior, 9, 9);
	MAT_INIT(P_post, 9, 9);
	MAT_INIT(V_accel, 3, 3);
	MAT_INIT(V_mag, 3, 3);
	MAT_INIT(F_x_t, 9, 9);
	MAT_INIT(FP, 9, 9);
	MAT_INIT(FPFt, 9, 9);
	MAT_INIT(FQFt, 9, 9);
	MAT_INIT(Q_delta_theta, 4, 3);
	MAT_INIT(X_delta_x, 10, 9);
	MAT_INIT(H_x_accel, 3, 10);
	MAT_INIT(H_accel, 3, 9);
	MAT_INIT(H_accel_t, 9, 3);
	MAT_INIT(PHt_accel, 9, 3);
	MAT_INIT(HPHt_accel, 3, 3);
	MAT_INIT(HPHt_V_accel, 3, 3);
	MAT_INIT(HPHt_V_accel_inv, 3, 3);
	MAT_INIT(K_accel, 9, 3);
	MAT_INIT(KH_accel, 9, 9);
	MAT_INIT(I_KH_accel, 9, 9);
	MAT_INIT(y_accel, 3, 1);
	MAT_INIT(h_accel, 3, 1);
	MAT_INIT(accel_resid, 3, 1);
	MAT_INIT(H_x_mag, 3, 10);
	MAT_INIT(H_mag, 3, 9);
	MAT_INIT(H_mag_t, 9, 3);
	MAT_INIT(PHt_mag, 9, 3);
	MAT_INIT(HPHt_mag, 3, 3);
	MAT_INIT(HPHt_V_mag, 3, 3);
	MAT_INIT(HPHt_V_mag_inv, 3, 3);
	MAT_INIT(K_mag, 9, 3);
	MAT_INIT(KH_mag, 9, 9);
	MAT_INIT(I_KH_mag, 9, 9);
	MAT_INIT(y_mag, 3, 1);
	MAT_INIT(h_mag, 3, 1);
	MAT_INIT(mag_resid, 3, 1);

	/* initialize the nominal state */
	mat_data(x_nominal)[0] = 0.0f; //px
	mat_data(x_nominal)[1] = 0.0f; //py
	mat_data(x_nominal)[2] = 0.0f; //pz
	mat_data(x_nominal)[3] = 0.0f; //vx
	mat_data(x_nominal)[4] = 0.0f; //vy
	mat_data(x_nominal)[5] = 0.0f; //vz
	mat_data(x_nominal)[6] = 1.0f; //q0
	mat_data(x_nominal)[7] = 0.0f; //q1
	mat_data(x_nominal)[8] = 0.0f; //q2
	mat_data(x_nominal)[9] = 0.0f; //q3

	/* initialize Q_i matrix */
	matrix_reset(mat_data(Q_i), 6, 6);
	mat_data(Q_i)[0*6 + 0] = ESKF_RESCALE(1e-5); //Var(ax)
	mat_data(Q_i)[1*6 + 1] = ESKF_RESCALE(1e-5); //Var(ay)
	mat_data(Q_i)[2*6 + 2] = ESKF_RESCALE(1e-5); //Var(az)
	mat_data(Q_i)[3*6 + 3] = ESKF_RESCALE(1e-5); //Var(wx)
	mat_data(Q_i)[4*6 + 4] = ESKF_RESCALE(1e-5); //Var(wy)
	mat_data(Q_i)[5*6 + 5] = ESKF_RESCALE(1e-5); //Var(wz)

	/* initialize P matrix */
	matrix_reset(mat_data(P_post), 9, 9);
	mat_data(P_post)[0*9 + 0] = ESKF_RESCALE(5.0f); //Var(px)
	mat_data(P_post)[1*9 + 1] = ESKF_RESCALE(5.0f); //Var(py)
	mat_data(P_post)[2*9 + 2] = ESKF_RESCALE(5.0f); //Var(pz)
	mat_data(P_post)[3*9 + 3] = ESKF_RESCALE(5.0f); //Var(vx)
	mat_data(P_post)[4*9 + 4] = ESKF_RESCALE(5.0f); //Var(vy)
	mat_data(P_post)[5*9 + 5] = ESKF_RESCALE(5.0f); //Var(vz)
	mat_data(P_post)[6*9 + 6] = ESKF_RESCALE(5.0f); //Var(theta_x)
	mat_data(P_post)[7*9 + 7] = ESKF_RESCALE(5.0f); //Var(theta_y)
	mat_data(P_post)[8*9 + 8] = ESKF_RESCALE(5.0f); //Var(theta_z)

	/* initialize V_accel matrix */
	matrix_reset(mat_data(V_accel), 9, 9);
	mat_data(V_accel)[0*3 + 0] = ESKF_RESCALE(7e-1); //Var(gx)
	mat_data(V_accel)[1*3 + 1] = ESKF_RESCALE(7e-1); //Var(gy)
	mat_data(V_accel)[2*3 + 2] = ESKF_RESCALE(7e-1); //Var(gz)

	/* initialize V_mag matrix */
	matrix_reset(mat_data(V_mag), 9, 9);
	mat_data(V_mag)[0*3 + 0] = ESKF_RESCALE(3); //Var(mx)
	mat_data(V_mag)[1*3 + 1] = ESKF_RESCALE(3); //Var(my)
	mat_data(V_mag)[2*3 + 2] = ESKF_RESCALE(3); //Var(mz)

	/* initial V_gps matrix */
	//TODO

	/* initialize V_barometer matrix */
	//TODO

	//FIXME: precalculation of FQFt
	int r, c;
	for(r = 0; r < 3; r++) {
		for(c = 0; c < 3; c++) {
			mat_data(FQFt)[r*3 + c] = mat_data(Q_i)[r*3 + c];
		}
	}
}

void eskf_ins_predict(float *gyro)
{
	float accel_i_ned[3] = {0}; //accelerometer reading in inertial frame
	//XXX: coordinate transform

	//convert acceleration from ned frame to enu frame
	float accel_i[3];
	accel_i[0] =  accel_i_ned[1];
	accel_i[1] =  accel_i_ned[0];
	accel_i[2] = -accel_i_ned[2];

	/* update nominal state (quaternion integration) */
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[0];
	w[2] = gyro[1];
	w[3] = gyro[2];

	float q_dot[4];
	quaternion_mult(w, &mat_data(x_nominal)[6], q_dot);

	//position integration
	mat_data(x_nominal)[0] += (mat_data(x_nominal)[3] * dt) +
	                          (accel_i[0] * half_dt_squared);
	mat_data(x_nominal)[1] += (mat_data(x_nominal)[4] * dt) +
	                          (accel_i[1] * half_dt_squared);
	mat_data(x_nominal)[2] += (mat_data(x_nominal)[5] * dt) +
	                          (accel_i[2] * half_dt_squared);
	//velocity integration
	mat_data(x_nominal)[0] += accel_i[0] * dt;
	mat_data(x_nominal)[1] += accel_i[1] * dt;
	mat_data(x_nominal)[2] += accel_i[2] * dt;
	//quaternion integration
	mat_data(x_nominal)[6] = mat_data(x_nominal)[6] + (q_dot[0] * neg_half_dt);
	mat_data(x_nominal)[7] = mat_data(x_nominal)[7] + (q_dot[1] * neg_half_dt);
	mat_data(x_nominal)[8] = mat_data(x_nominal)[8] + (q_dot[2] * neg_half_dt);
	mat_data(x_nominal)[9] = mat_data(x_nominal)[9] + (q_dot[3] * neg_half_dt);
	quat_normalize(mat_data(x_nominal));

	/* construct error state transition matrix */
	//FIXME
	mat_data(F_x)[0*3 + 0] = dt;
	mat_data(F_x)[0*3 + 1] = +gyro[2] * dt;
	mat_data(F_x)[0*3 + 2] = -gyro[1] * dt;

	mat_data(F_x)[1*3 + 0] = -gyro[2] * dt;
	mat_data(F_x)[1*3 + 1] = dt;
	mat_data(F_x)[1*3 + 2] = +gyro[0];

	mat_data(F_x)[2*3 + 0] = +gyro[1] * dt;
	mat_data(F_x)[2*3 + 1] = -gyro[0] * dt;
	mat_data(F_x)[2*3 + 2] = dt;

	/* update process noise covariance matrix */
	MAT_TRANS(&F_x, &F_x_t)
	MAT_MULT(&F_x, &P_post, &FP);
	MAT_MULT(&FP, &F_x_t, &FPFt);
	MAT_ADD(&FPFt, &FQFt, &P_prior);
}

void eskf_ins_accelerometer_correct(float *accel)
{
	mat_data(y_accel)[0] = accel[0];
	mat_data(y_accel)[1] = accel[1];
	mat_data(y_accel)[2] = accel[2];

	float q0 = mat_data(x_nominal)[6];
	float q1 = mat_data(x_nominal)[7];
	float q2 = mat_data(x_nominal)[8];
	float q3 = mat_data(x_nominal)[9];

	/* construct error state observation matrix */
	//FIXME
	mat_data(H_x_accel)[0*4 + 0] = +2*q2;
	mat_data(H_x_accel)[0*4 + 1] = +2*q3;
	mat_data(H_x_accel)[0*4 + 2] = +2*q0;
	mat_data(H_x_accel)[0*4 + 3] = +2*q1;

	mat_data(H_x_accel)[1*4 + 0] = -2*q1;
	mat_data(H_x_accel)[1*4 + 1] = -2*q0;
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
	mat_data(h_accel)[0] = 2 * (q0*q2 + q1*q3);
	mat_data(h_accel)[1] = 2 * (q2*q3 - q0*q1);
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

void eskf_ins_magnetometer_correct(float *mag)
{
	mat_data(y_mag)[0] = mag[0];
	mat_data(y_mag)[1] = mag[1];
	mat_data(y_mag)[2] = mag[2];

	float q0 = mat_data(x_nominal)[0];
	float q1 = mat_data(x_nominal)[1];
	float q2 = mat_data(x_nominal)[2];
	float q3 = mat_data(x_nominal)[3];

	float gamma = sqrt(mag[0]*mag[0] + mag[1]*mag[1]);

	/* construct error state observation matrix */
	mat_data(H_x_mag)[0*4 + 0] = 2*(+gamma*q0 + mag[2]*q2);
	mat_data(H_x_mag)[0*4 + 1] = 2*(+gamma*q1 + mag[2]*q3);
	mat_data(H_x_mag)[0*4 + 2] = 2*(-gamma*q2 + mag[2]*q0);
	mat_data(H_x_mag)[0*4 + 3] = 2*(-gamma*q3 + mag[2]*q1);

	mat_data(H_x_mag)[1*4 + 0] = 2*(+gamma*q3 - mag[2]*q1);
	mat_data(H_x_mag)[1*4 + 1] = 2*(+gamma*q2 - mag[2]*q0);
	mat_data(H_x_mag)[1*4 + 2] = 2*(+gamma*q1 + mag[2]*q3);
	mat_data(H_x_mag)[1*4 + 3] = 2*(+gamma*q0 + mag[2]*q2);

	mat_data(H_x_mag)[2*4 + 0] = 2*(-gamma*q2 + mag[2]*q0);
	mat_data(H_x_mag)[2*4 + 1] = 2*(+gamma*q3 - mag[2]*q1);
	mat_data(H_x_mag)[2*4 + 2] = 2*(-gamma*q0 - mag[2]*q2);
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
	mat_data(h_mag)[0] = gamma*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2*mag[2]*(q0*q2 + q1*q3);
	mat_data(h_mag)[1] = 2*gamma*(q1*q2 + q0*q3) + 2*mag[2]*(q2*q3 - q0*q1);
	mat_data(h_mag)[2] = 2*gamma*(q1*q3 - q0*q2) + mag[2]*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

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

void eskf_ins_gps_correct(float longitude, float latitude,
                          float vx_ned, float vy_ned)
{
}

void eskf_ins_barometer_correct(float barometer_z, float barometer_vz)
{
}

void get_eskf_ins_attitude_quaternion(float *q_out)
{
	/* return the conjugated quaternion since we use opposite convention compared to the paper.
	 * paper: quaternion of earth frame to body-fixed frame
	 * us: quaternion of body-fixed frame to earth frame */
	quaternion_conj(mat_data(x_nominal), q_out);
}
