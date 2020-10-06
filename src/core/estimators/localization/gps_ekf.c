#include "arm_math.h"
#include "matrix.h"

#define EARTH_RADIUS 6371.0f

MAT_ALLOC(accel_b, 3, 1);
MAT_ALLOC(accel_i, 3, 1);
MAT_ALLOC(x_priori, 6, 1);
MAT_ALLOC(x_posteriori, 6, 1);
MAT_ALLOC(x_dot, 6, 1);
MAT_ALLOC(x_dot_dt, 6, 1);
MAT_ALLOC(z, 6, 1);
MAT_ALLOC(u, 3, 1);
MAT_ALLOC(z, 6, 1);
MAT_ALLOC(A, 6, 6);
MAT_ALLOC(At, 6, 6);
MAT_ALLOC(B, 6, 3);
MAT_ALLOC(Ax, 6, 1);
MAT_ALLOC(Bu, 6, 1);
MAT_ALLOC(H, 6, 6);
MAT_ALLOC(Ht, 6, 6);
MAT_ALLOC(P_priori, 6, 1);
MAT_ALLOC(P_posteriori, 6, 6);
MAT_ALLOC(P_dot, 6, 6);
MAT_ALLOC(P_dot_dt, 6, 6);
MAT_ALLOC(AP, 6, 6);
MAT_ALLOC(PAt, 6, 6);
MAT_ALLOC(AP_PAt, 6, 6);
MAT_ALLOC(Q, 6, 6);
MAT_ALLOC(R, 6, 6);
MAT_ALLOC(K, 6, 6);
MAT_ALLOC(HPt, 6, 6);
MAT_ALLOC(HPHt, 6, 6);
MAT_ALLOC(HPHt_R, 6, 6);
MAT_ALLOC(HPHt_R_inv, 6, 6);
MAT_ALLOC(PHt, 6, 6);
MAT_ALLOC(residual, 6, 1);
MAT_ALLOC(K_z_Hx, 6, 1);
MAT_ALLOC(I, 6, 6);
MAT_ALLOC(KH, 6, 6);
MAT_ALLOC(I_KH, 6, 6);

float dt;

float home_longitude = 0.0f;
float home_latitude = 0.0f;

float home_ecef_x = 0.0f;
float home_ecef_y = 0.0f;

void set_home_longitude_latitude(float longitude, float latitude)
{
	float sin_lambda = arm_sin_f32(longitude);
	float cos_lambda = arm_cos_f32(longitude);
	float cos_phi = arm_cos_f32(latitude);

	home_longitude = longitude;
	home_latitude = latitude;

	home_ecef_x = EARTH_RADIUS * cos_phi * cos_lambda;
	home_ecef_y = EARTH_RADIUS * cos_phi * sin_lambda;
	//home_ecef_z = EARTH_RADIUS * sin_phi
}

void get_home_longitude_latitude(float *longitude, float *latitude)
{
	*longitude = home_longitude;
	*latitude = home_latitude;
}

void longitude_latitude_to_enu(float longitude, float latitude,
                               float *x_enu, float *y_enu)
{
	float sin_lambda = arm_sin_f32(longitude);
	float cos_lambda = arm_cos_f32(longitude);
	float sin_phi = arm_sin_f32(latitude);
	float cos_phi = arm_cos_f32(latitude);

	/* convert geodatic coordinates to earth center earth fixed frame (ecef) */
	float ecef_now_x = EARTH_RADIUS * cos_phi * cos_lambda;
	float ecef_now_y = EARTH_RADIUS * cos_phi * sin_lambda;
	//float ecef_now_z = EARTH_RADIUS * sin_phi;

	/* convert position from earth center earth fixed frame to east north up frame */
	float r11 = -sin_lambda;
	float r12 = -cos_lambda * sin_phi;
	//float r13 = cos_lambda * cos_phi;
	float r21 = cos_lambda;
	float r22 = -sin_lambda * sin_phi;
	//float r23 = sin_lambda * cos_phi;
	//float r31 = 0.0f;
	//float r32 = cos_phi;
	//float r33 = sin_phi;

	float dx = ecef_now_x - home_ecef_x;
	float dy = ecef_now_y - home_ecef_y;
	//float dz = ecef_now_z - home_ecef_z;

	*x_enu = (r11 * dx) + (r12 * dy);
	*y_enu = (r21 * dx) + (r22 * dy);
}

void gps_ekf_init(float _dt)
{
	dt = _dt;

	MAT_INIT(accel_b, 3, 1);
	MAT_INIT(accel_i, 3, 1);
	MAT_INIT(x_priori, 6, 1);
	MAT_INIT(x_posteriori, 6, 1);
	MAT_INIT(x_dot, 6, 1);
	MAT_INIT(x_dot_dt, 6, 1);
	MAT_INIT(z, 6, 1);
	MAT_INIT(u, 3, 1);
	MAT_INIT(z, 6, 1);
	MAT_INIT(A, 6, 6);
	MAT_INIT(At, 6, 6);
	MAT_INIT(B, 6, 3);
	MAT_INIT(Ax, 6, 1);
	MAT_INIT(Bu, 6, 1);
	MAT_INIT(H, 6, 6);
	MAT_INIT(Ht, 6, 6);
	MAT_INIT(P_priori, 6, 1);
	MAT_INIT(P_posteriori, 6, 6);
	MAT_INIT(P_dot, 6, 6);
	MAT_INIT(P_dot_dt, 6, 6);
	MAT_INIT(AP, 6, 6);
	MAT_INIT(PAt, 6, 6);
	MAT_INIT(AP_PAt, 6, 6);
	MAT_INIT(Q, 6, 6);
	MAT_INIT(R, 6, 6);
	MAT_INIT(K, 6, 6);
	MAT_INIT(HPt, 6, 6);
	MAT_INIT(HPHt, 6, 6);
	MAT_INIT(HPHt_R, 6, 6);
	MAT_INIT(HPHt_R_inv, 6, 6);
	MAT_INIT(PHt, 6, 6);
	MAT_INIT(residual, 6, 1);
	MAT_INIT(K_z_Hx, 6, 1);
	MAT_INIT(I, 6, 6);
	MAT_INIT(KH, 6, 6);
	MAT_INIT(I_KH, 6, 6);

	MAT_TRANS(&A, &At);
	MAT_TRANS(&H, &Ht);

	/* init 6x6 identity matrix */
	mat_data(I)[0*6 + 0] = 1.0f;
	mat_data(I)[1*6 + 1] = 1.0f;
	mat_data(I)[2*6 + 2] = 1.0f;
	mat_data(I)[3*6 + 3] = 1.0f;
	mat_data(I)[4*6 + 4] = 1.0f;
	mat_data(I)[5*6 + 5] = 1.0f;

	/* init state estimate covariance matrix [m^2] */
	mat_data(P_posteriori)[0*6 + 0] = 10.0f;
	mat_data(P_posteriori)[1*6 + 1] = 10.0f;
	mat_data(P_posteriori)[2*6 + 2] = 10.0f;
	mat_data(P_posteriori)[3*6 + 3] = 10.0f;
	mat_data(P_posteriori)[4*6 + 4] = 10.0f;
	mat_data(P_posteriori)[5*6 + 5] = 10.0f;

	/* init process noise covariance matrix [m^2] */
	mat_data(Q)[0*6 + 0] = 0.5f;
	mat_data(Q)[1*6 + 1] = 0.5f;
	mat_data(Q)[2*6 + 2] = 0.5f;
	mat_data(Q)[3*6 + 3] = 0.5f;
	mat_data(Q)[4*6 + 4] = 0.5f;
	mat_data(Q)[5*6 + 5] = 0.5f;

	/* init observation noise covariance matrix [m^2] */
	mat_data(R)[0*6 + 0] = 2.0f;
	mat_data(R)[1*6 + 1] = 2.0f;
	mat_data(R)[2*6 + 2] = 2.0f;
	mat_data(R)[3*6 + 3] = 2.0f;
	mat_data(R)[4*6 + 4] = 2.0f;
	mat_data(R)[5*6 + 5] = 2.0f;
}

void gps_ekf_predict(void)
{
	/* read accelerometer */
	//TODO: read it

	/* read transposed attitude direction cosine matrix */
	arm_matrix_instance_f32 *Rt = NULL;
	//TODO: read it

	/* transform acceleration from body-fixed frame to earth frame and
	 * cancel the gravitational acceleration */
	MAT_MULT(Rt, &accel_b, &accel_i);
	mat_data(u)[0] = mat_data(accel_i)[0];
	mat_data(u)[1] = mat_data(accel_i)[1];
	mat_data(u)[2] = mat_data(accel_i)[2] - 9.81f;

	/* calculate state derivative */
	//x_dot = Ax + Bu
	MAT_MULT(&A, &x_posteriori, &Ax);
	MAT_MULT(&B, &u, &Bu);
	MAT_ADD(&Ax, &Bu, &x_dot);

	/* calculate derivative of state estimate covariance matrix */
	//P_dot = AP + PAt + Q
	MAT_MULT(&A, &P_posteriori, &AP);
	MAT_MULT(&P_posteriori, &At, &PAt);
	MAT_MULT(&AP, &PAt, &AP_PAt);
	MAT_ADD(&AP_PAt, &Q, &P_dot);

	/* calculate a priori state */
	//x_priori = x_last + (dt * a_dot)
	MAT_SCALE(&x_dot, dt, &x_dot_dt);
	MAT_ADD(&x_posteriori, &x_dot_dt, &x_priori);

	/* calculate a priori state estimate covariance matrix */
	//P_priori = P_last + (dt * P_dot)
	MAT_SCALE(&P_dot, dt, &P_dot_dt);
	MAT_ADD(&P_posteriori, &P_dot_dt, &P_priori);
}

void gps_ekf_update(void)
{
	/* read longitude, latitude from gps receiver and height from height sensor */
	float longitude = 0.0f, latitude = 0.0f, height_msl = 0.0f;
	//TODO: read it

	float x_enu = 0.0f, y_enu = 0.0f;
	longitude_latitude_to_enu(longitude, latitude, &x_enu, &y_enu);

	mat_data(z)[0] = x_enu;
	mat_data(z)[1] = y_enu;
	mat_data(z)[2] = height_msl;

	/* calculate kalman gain */
	//K = PHt * (HPHt + R)^-1
	MAT_MULT(&P_priori, &Ht, &PHt);
	MAT_MULT(&H, &PHt, &HPHt);
	MAT_ADD(&HPHt, &R, &HPHt_R);
	MAT_INV(&HPHt_R, &HPHt_R_inv);
	MAT_MULT(&PHt, &HPHt_R_inv, &K);

	/* calculate a posteriori state */
	//x_posteriori = x_priori + K * [z - (H * x_posteriori)]
	MAT_SUB(&z, &x_priori, &residual);
	MAT_MULT(&K, &residual, &K_z_Hx);
	MAT_ADD(&x_priori, &K_z_Hx, &x_posteriori);

	/* calculate a posteriori state estimate covariance matrix */
	//P_posteriori = [I - KH] * P_priori
	MAT_MULT(&K, &H, &KH);
	MAT_SUB(&I, &KH, &I_KH);
	MAT_MULT(&I_KH, &P_priori, &P_posteriori);
}
