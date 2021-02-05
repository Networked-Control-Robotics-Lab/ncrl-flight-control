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

#define P_prior(r, c)      _P_prior.pData[(r * 9) + c]
#define P_post(r, c)       _P_post.pData[(r * 9) + c]
#define R_am_ab_dt(r, c)   _R_am_ab_dt.pData[(r * 3) + c]
#define Rt_wm_wb_dt(r, c)  _Rt_wm_wb_dt.pData[(r * 3) + c]
#define Q_i(r, c)          _Q_i.pData[(r * 6) + c]

MAT_ALLOC(x_nominal, 10, 1);
MAT_ALLOC(x_error_state, 9, 1);
MAT_ALLOC(F_x, 9, 9);
MAT_ALLOC(_Q_i, 6, 6);
MAT_ALLOC(_P_prior, 9, 9);
MAT_ALLOC(_P_post, 9, 9);
MAT_ALLOC(V_accel, 3, 3);
MAT_ALLOC(V_mag, 3, 3);
MAT_ALLOC(F_x_t, 9, 9);
MAT_ALLOC(FP, 9, 9);
MAT_ALLOC(FPFt, 9, 9);
MAT_ALLOC(FQFt, 9, 9);
MAT_ALLOC(_R_am_ab_dt, 3, 3);
MAT_ALLOC(_Rt_wm_wb_dt, 3, 3);
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
	MAT_INIT(_Q_i, 6, 6);
	MAT_INIT(_P_prior, 9, 9);
	MAT_INIT(_P_post, 9, 9);
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

	/* initialize _Q_i matrix */
	matrix_reset(mat_data(_Q_i), 6, 6);
	mat_data(_Q_i)[0*6 + 0] = ESKF_RESCALE(1e-5); //Var(ax)
	mat_data(_Q_i)[1*6 + 1] = ESKF_RESCALE(1e-5); //Var(ay)
	mat_data(_Q_i)[2*6 + 2] = ESKF_RESCALE(1e-5); //Var(az)
	mat_data(_Q_i)[3*6 + 3] = ESKF_RESCALE(1e-5); //Var(wx)
	mat_data(_Q_i)[4*6 + 4] = ESKF_RESCALE(1e-5); //Var(wy)
	mat_data(_Q_i)[5*6 + 5] = ESKF_RESCALE(1e-5); //Var(wz)

	/* initialize P matrix */
	matrix_reset(mat_data(_P_post), 9, 9);
	mat_data(_P_post)[0*9 + 0] = ESKF_RESCALE(5.0f); //Var(px)
	mat_data(_P_post)[1*9 + 1] = ESKF_RESCALE(5.0f); //Var(py)
	mat_data(_P_post)[2*9 + 2] = ESKF_RESCALE(5.0f); //Var(pz)
	mat_data(_P_post)[3*9 + 3] = ESKF_RESCALE(5.0f); //Var(vx)
	mat_data(_P_post)[4*9 + 4] = ESKF_RESCALE(5.0f); //Var(vy)
	mat_data(_P_post)[5*9 + 5] = ESKF_RESCALE(5.0f); //Var(vz)
	mat_data(_P_post)[6*9 + 6] = ESKF_RESCALE(5.0f); //Var(theta_x)
	mat_data(_P_post)[7*9 + 7] = ESKF_RESCALE(5.0f); //Var(theta_y)
	mat_data(_P_post)[8*9 + 8] = ESKF_RESCALE(5.0f); //Var(theta_z)

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
			mat_data(FQFt)[r*3 + c] = mat_data(_Q_i)[r*3 + c];
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

	/* calculate a priori process covariance matrix */
	float c0 = P_post(5,8)+P_post(6,8)*R_am_ab_dt(2,0)+P_post(7,8)*R_am_ab_dt(2,1)+P_post(8,8)*R_am_ab_dt(2,2);
	float c1 = P_post(5,7)+P_post(6,7)*R_am_ab_dt(2,0)+P_post(7,7)*R_am_ab_dt(2,1)+P_post(8,7)*R_am_ab_dt(2,2);
	float c2 = P_post(5,6)+P_post(6,6)*R_am_ab_dt(2,0)+P_post(7,6)*R_am_ab_dt(2,1)+P_post(8,6)*R_am_ab_dt(2,2);
	float c3 = P_post(4,8)+P_post(6,8)*R_am_ab_dt(1,0)+P_post(7,8)*R_am_ab_dt(1,1)+P_post(8,8)*R_am_ab_dt(1,2);
	float c4 = P_post(4,7)+P_post(6,7)*R_am_ab_dt(1,0)+P_post(7,7)*R_am_ab_dt(1,1)+P_post(8,7)*R_am_ab_dt(1,2);
	float c5 = P_post(4,6)+P_post(6,6)*R_am_ab_dt(1,0)+P_post(7,6)*R_am_ab_dt(1,1)+P_post(8,6)*R_am_ab_dt(1,2);
	float c6 = P_post(3,8)+P_post(6,8)*R_am_ab_dt(0,0)+P_post(7,8)*R_am_ab_dt(0,1)+P_post(8,8)*R_am_ab_dt(0,2);
	float c7 = P_post(3,7)+P_post(6,7)*R_am_ab_dt(0,0)+P_post(7,7)*R_am_ab_dt(0,1)+P_post(8,7)*R_am_ab_dt(0,2);
	float c8 = P_post(3,6)+P_post(6,6)*R_am_ab_dt(0,0)+P_post(7,6)*R_am_ab_dt(0,1)+P_post(8,6)*R_am_ab_dt(0,2);
	float c9 = P_post(6,8)*Rt_wm_wb_dt(2,0)+P_post(7,8)*Rt_wm_wb_dt(2,1)+P_post(8,8)*Rt_wm_wb_dt(2,2);
	float c10 = P_post(6,7)*Rt_wm_wb_dt(2,0)+P_post(7,7)*Rt_wm_wb_dt(2,1)+P_post(8,7)*Rt_wm_wb_dt(2,2);
	float c11 = P_post(6,6)*Rt_wm_wb_dt(2,0)+P_post(7,6)*Rt_wm_wb_dt(2,1)+P_post(8,6)*Rt_wm_wb_dt(2,2);
	float c12 = P_post(6,8)*Rt_wm_wb_dt(1,0)+P_post(7,8)*Rt_wm_wb_dt(1,1)+P_post(8,8)*Rt_wm_wb_dt(1,2);
	float c13 = P_post(6,7)*Rt_wm_wb_dt(1,0)+P_post(7,7)*Rt_wm_wb_dt(1,1)+P_post(8,7)*Rt_wm_wb_dt(1,2);
	float c14 = P_post(6,6)*Rt_wm_wb_dt(1,0)+P_post(7,6)*Rt_wm_wb_dt(1,1)+P_post(8,6)*Rt_wm_wb_dt(1,2);
	float c15 = P_post(6,8)*Rt_wm_wb_dt(0,0)+P_post(7,8)*Rt_wm_wb_dt(0,1)+P_post(8,8)*Rt_wm_wb_dt(0,2);
	float c16 = P_post(6,7)*Rt_wm_wb_dt(0,0)+P_post(7,7)*Rt_wm_wb_dt(0,1)+P_post(8,7)*Rt_wm_wb_dt(0,2);
	float c17 = P_post(6,6)*Rt_wm_wb_dt(0,0)+P_post(7,6)*Rt_wm_wb_dt(0,1)+P_post(8,6)*Rt_wm_wb_dt(0,2);
	float c18 = P_post(2,8)+P_post(5,8)*dt;
	float c19 = P_post(2,7)+P_post(5,7)*dt;
	float c20 = P_post(2,6)+P_post(5,6)*dt;
	float c21 = P_post(1,8)+P_post(4,8)*dt;
	float c22 = P_post(1,7)+P_post(4,7)*dt;
	float c23 = P_post(1,6)+P_post(4,6)*dt;
	float c24 = P_post(0,8)+P_post(3,8)*dt;
	float c25 = P_post(0,7)+P_post(3,7)*dt;
	float c26 = P_post(0,6)+P_post(3,6)*dt;
	float c27 = P_post(5,5)*dt;
	float c28 = P_post(5,4)*dt;
	float c29 = P_post(5,3)*dt;
	float c30 = P_post(4,5)*dt;
	float c31 = P_post(4,4)*dt;
	float c32 = P_post(4,3)*dt;
	float c33 = P_post(3,5)*dt;
	float c34 = P_post(3,4)*dt;
	float c35 = P_post(3,3)*dt;
	float c63 = P_post(8,5)*R_am_ab_dt(2,2);
	float c66 = P_post(8,5)*R_am_ab_dt(1,2);
	float c67 = P_post(8,4)*R_am_ab_dt(1,2);
	float c68 = P_post(7,5)*R_am_ab_dt(2,1);
	float c72 = P_post(8,5)*R_am_ab_dt(0,2);
	float c73 = P_post(8,4)*R_am_ab_dt(0,2);
	float c74 = P_post(7,5)*R_am_ab_dt(1,1);
	float c75 = P_post(8,3)*R_am_ab_dt(0,2);
	float c76 = P_post(7,4)*R_am_ab_dt(1,1);
	float c77 = P_post(6,5)*R_am_ab_dt(2,0);
	float c81 = P_post(7,5)*R_am_ab_dt(0,1);
	float c82 = P_post(7,4)*R_am_ab_dt(0,1);
	float c83 = P_post(6,5)*R_am_ab_dt(1,0);
	float c84 = P_post(7,3)*R_am_ab_dt(0,1);
	float c85 = P_post(6,4)*R_am_ab_dt(1,0);
	float c87 = P_post(6,5)*R_am_ab_dt(0,0);
	float c88 = P_post(6,4)*R_am_ab_dt(0,0);
	float c89 = P_post(6,3)*R_am_ab_dt(0,0);
	float c90 = P_post(3,3)+c75+c84+c89;
	float c91 = P_post(3,4)+c73+c82+c88;
	float c93 = P_post(3,5)+c72+c81+c87;
	float c94 = P_post(4,4)+c67+c76+c85;
	float c96 = P_post(4,5)+c66+c74+c83;
	float c98 = P_post(5,5)+c63+c68+c77;
	float c108 = P_post(2,5)+c27;
	float c109 = P_post(2,4)+c28;
	float c110 = P_post(2,3)+c29;
	float c111 = P_post(1,5)+c30;
	float c112 = P_post(1,4)+c31;
	float c113 = P_post(1,3)+c32;
	float c114 = P_post(0,5)+c33;
	float c115 = P_post(0,4)+c34;
	float c116 = P_post(0,3)+c35;

	P_prior(0, 0) = P_post(0,0)+P_post(3,0)*dt+c116*dt;
	P_prior(0, 1) = P_post(0,1)+P_post(3,1)*dt+c115*dt;
	P_prior(0, 2) = P_post(0,2)+P_post(3,2)*dt+c114*dt;
	P_prior(0, 3) = c116+R_am_ab_dt(0,0)*c26+R_am_ab_dt(0,1)*c25+R_am_ab_dt(0,2)*c24;
	P_prior(0, 4) = c115+R_am_ab_dt(1,0)*c26+R_am_ab_dt(1,1)*c25+R_am_ab_dt(1,2)*c24;
	P_prior(0, 5) = c114+R_am_ab_dt(2,0)*c26+R_am_ab_dt(2,1)*c25+R_am_ab_dt(2,2)*c24;
	P_prior(0, 6) = Rt_wm_wb_dt(0,0)*c26+Rt_wm_wb_dt(0,1)*c25+Rt_wm_wb_dt(0,2)*c24;
	P_prior(0, 7) = Rt_wm_wb_dt(1,0)*c26+Rt_wm_wb_dt(1,1)*c25+Rt_wm_wb_dt(1,2)*c24;
	P_prior(0, 8) = Rt_wm_wb_dt(2,0)*c26+Rt_wm_wb_dt(2,1)*c25+Rt_wm_wb_dt(2,2)*c24;
	P_prior(1, 0) = P_prior(0, 1);
	P_prior(1, 1) = P_post(1,1)+P_post(4,1)*dt+c112*dt;
	P_prior(1, 2) = P_post(1,2)+P_post(4,2)*dt+c111*dt;
	P_prior(1, 3) = c113+R_am_ab_dt(0,0)*c23+R_am_ab_dt(0,1)*c22+R_am_ab_dt(0,2)*c21;
	P_prior(1, 4) = c112+R_am_ab_dt(1,0)*c23+R_am_ab_dt(1,1)*c22+R_am_ab_dt(1,2)*c21;
	P_prior(1, 5) = c111+R_am_ab_dt(2,0)*c23+R_am_ab_dt(2,1)*c22+R_am_ab_dt(2,2)*c21;
	P_prior(1, 6) = Rt_wm_wb_dt(0,0)*c23+Rt_wm_wb_dt(0,1)*c22+Rt_wm_wb_dt(0,2)*c21;
	P_prior(1, 7) = Rt_wm_wb_dt(1,0)*c23+Rt_wm_wb_dt(1,1)*c22+Rt_wm_wb_dt(1,2)*c21;
	P_prior(1, 8) = Rt_wm_wb_dt(2,0)*c23+Rt_wm_wb_dt(2,1)*c22+Rt_wm_wb_dt(2,2)*c21;
	P_prior(2, 0) = P_prior(0, 2);
	P_prior(2, 1) = P_prior(1, 2);
	P_prior(2, 2) = P_post(2,2)+P_post(5,2)*dt+c108*dt;
	P_prior(2, 3) = c110+R_am_ab_dt(0,0)*c20+R_am_ab_dt(0,1)*c19+R_am_ab_dt(0,2)*c18;
	P_prior(2, 4) = c109+R_am_ab_dt(1,0)*c20+R_am_ab_dt(1,1)*c19+R_am_ab_dt(1,2)*c18;
	P_prior(2, 5) = c108+R_am_ab_dt(2,0)*c20+R_am_ab_dt(2,1)*c19+R_am_ab_dt(2,2)*c18;
	P_prior(2, 6) = Rt_wm_wb_dt(0,0)*c20+Rt_wm_wb_dt(0,1)*c19+Rt_wm_wb_dt(0,2)*c18;
	P_prior(2, 7) = Rt_wm_wb_dt(1,0)*c20+Rt_wm_wb_dt(1,1)*c19+Rt_wm_wb_dt(1,2)*c18;
	P_prior(2, 8) = Rt_wm_wb_dt(2,0)*c20+Rt_wm_wb_dt(2,1)*c19+Rt_wm_wb_dt(2,2)*c18;
	P_prior(3, 0) = P_prior(0, 3);
	P_prior(3, 1) = P_prior(1, 3);
	P_prior(3, 2) = P_prior(2, 3);
	P_prior(3, 3) = Q_i(0,0)+c90+R_am_ab_dt(0,0)*c8+R_am_ab_dt(0,1)*c7+R_am_ab_dt(0,2)*c6;
	P_prior(3, 4) = c91+R_am_ab_dt(1,0)*c8+R_am_ab_dt(1,1)*c7+R_am_ab_dt(1,2)*c6;
	P_prior(3, 5) = c93+R_am_ab_dt(2,0)*c8+R_am_ab_dt(2,1)*c7+R_am_ab_dt(2,2)*c6;
	P_prior(3, 6) = Rt_wm_wb_dt(0,0)*c8+Rt_wm_wb_dt(0,1)*c7+Rt_wm_wb_dt(0,2)*c6;
	P_prior(3, 7) = Rt_wm_wb_dt(1,0)*c8+Rt_wm_wb_dt(1,1)*c7+Rt_wm_wb_dt(1,2)*c6;
	P_prior(3, 8) = Rt_wm_wb_dt(2,0)*c8+Rt_wm_wb_dt(2,1)*c7+Rt_wm_wb_dt(2,2)*c6;
	P_prior(4, 0) = P_prior(0, 4);
	P_prior(4, 1) = P_prior(1, 4);
	P_prior(4, 2) = P_prior(2, 4);
	P_prior(4, 3) = P_prior(3, 4);
	P_prior(4, 4) = Q_i(1,1)+c94+R_am_ab_dt(1,0)*c5+R_am_ab_dt(1,1)*c4+R_am_ab_dt(1,2)*c3;
	P_prior(4, 5) = c96+R_am_ab_dt(2,0)*c5+R_am_ab_dt(2,1)*c4+R_am_ab_dt(2,2)*c3;
	P_prior(4, 6) = Rt_wm_wb_dt(0,0)*c5+Rt_wm_wb_dt(0,1)*c4+Rt_wm_wb_dt(0,2)*c3;
	P_prior(4, 7) = Rt_wm_wb_dt(1,0)*c5+Rt_wm_wb_dt(1,1)*c4+Rt_wm_wb_dt(1,2)*c3;
	P_prior(4, 8) = Rt_wm_wb_dt(2,0)*c5+Rt_wm_wb_dt(2,1)*c4+Rt_wm_wb_dt(2,2)*c3;
	P_prior(5, 0) = P_prior(0, 5);
	P_prior(5, 1) = P_prior(1, 5);
	P_prior(5, 2) = P_prior(2, 5);
	P_prior(5, 3) = P_prior(3, 5);
	P_prior(5, 4) = P_prior(4, 5);
	P_prior(5, 5) = Q_i(2,2)+c98+R_am_ab_dt(2,0)*c2+R_am_ab_dt(2,1)*c1+R_am_ab_dt(2,2)*c0;
	P_prior(5, 6) = Rt_wm_wb_dt(0,0)*c2+Rt_wm_wb_dt(0,1)*c1+Rt_wm_wb_dt(0,2)*c0;
	P_prior(5, 7) = Rt_wm_wb_dt(1,0)*c2+Rt_wm_wb_dt(1,1)*c1+Rt_wm_wb_dt(1,2)*c0;
	P_prior(5, 8) = Rt_wm_wb_dt(2,0)*c2+Rt_wm_wb_dt(2,1)*c1+Rt_wm_wb_dt(2,2)*c0;
	P_prior(6, 0) = P_prior(0, 6);
	P_prior(6, 1) = P_prior(1, 6);
	P_prior(6, 2) = P_prior(2, 6);
	P_prior(6, 3) = P_prior(3, 6);
	P_prior(6, 4) = P_prior(4, 6);
	P_prior(6, 5) = P_prior(5, 6);
	P_prior(6, 6) = Q_i(3,3)+Rt_wm_wb_dt(0,0)*c17+Rt_wm_wb_dt(0,1)*c16+Rt_wm_wb_dt(0,2)*c15;
	P_prior(6, 7) = Rt_wm_wb_dt(1,0)*c17+Rt_wm_wb_dt(1,1)*c16+Rt_wm_wb_dt(1,2)*c15;
	P_prior(6, 8) = Rt_wm_wb_dt(2,0)*c17+Rt_wm_wb_dt(2,1)*c16+Rt_wm_wb_dt(2,2)*c15;
	P_prior(7, 0) = P_prior(0, 7);
	P_prior(7, 1) = P_prior(1, 7);
	P_prior(7, 2) = P_prior(2, 7);
	P_prior(7, 3) = P_prior(3, 7);
	P_prior(7, 4) = P_prior(4, 7);
	P_prior(7, 5) = P_prior(5, 7);
	P_prior(7, 6) = P_prior(6, 7);
	P_prior(7, 7) = Q_i(4,4)+Rt_wm_wb_dt(1,0)*c14+Rt_wm_wb_dt(1,1)*c13+Rt_wm_wb_dt(1,2)*c12;
	P_prior(7, 8) = Rt_wm_wb_dt(2,0)*c14+Rt_wm_wb_dt(2,1)*c13+Rt_wm_wb_dt(2,2)*c12;
	P_prior(8, 0) = P_prior(0, 8);
	P_prior(8, 1) = P_prior(1, 8);
	P_prior(8, 2) = P_prior(2, 8);
	P_prior(8, 3) = P_prior(3, 8);
	P_prior(8, 4) = P_prior(4, 8);
	P_prior(8, 5) = P_prior(5, 8);
	P_prior(8, 6) = P_prior(6, 8);
	P_prior(8, 7) = P_prior(7, 8);
	P_prior(8, 8) = Q_i(5,5)+Rt_wm_wb_dt(2,0)*c11+Rt_wm_wb_dt(2,1)*c10+Rt_wm_wb_dt(2,2)*c9;
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
	MAT_MULT(&_P_prior, &H_accel_t, &PHt_accel);
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
	MAT_MULT(&I_KH_accel, &_P_prior, &_P_post);

	/* _P_post becoms the _P_prior of the magnatometer correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9);

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
	MAT_MULT(&_P_prior, &H_mag_t, &PHt_mag);
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
	MAT_MULT(&I_KH_mag, &_P_prior, &_P_post);

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
