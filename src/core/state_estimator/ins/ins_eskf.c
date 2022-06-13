#include <math.h>
#include "arm_math.h"
#include "ahrs.h"
#include "lpf.h"
#include "matrix.h"
#include "se3_math.h"
#include "quaternion.h"
#include "ins_sensor_sync.h"
#include "led.h"
#include "geographic_transform.h"
#include "optitrack.h"
#include "imu.h"
#include "proj_config.h"
#include "compass.h"
#include "gps.h"
#include "barometer.h"
#include "system_state.h"
#include "vins_mono.h"
#include "rangefinder.h"
#include "sys_time.h"
#include "ins_eskf.h"

#define ESKF_CONVERGENCE_NORM (5e-1)

#define R(r, c)                  _R.pData[(r * 3) + c]
#define Rt(r, c)                 _Rt.pData[(r * 3) + c]
#define P_prior(r, c)            _P_prior.pData[(r * 9) + c]
#define P_post(r, c)             _P_post.pData[(r * 9) + c]
#define R_am_ab_dt(r, c)         _R_am_ab_dt.pData[(r * 3) + c]
#define Rt_wm_wb_dt(r, c)        _Rt_wm_wb_dt.pData[(r * 3) + c]
#define Q_i(r, c)                _Q_i.pData[(r * 6) + c]
#define K_accel(r, c)            _K_accel.pData[(r * 3) + c]
#define K_mag(r, c)              _K_mag.pData[(r * 3) + c]
#define K_gps(r, c)              _K_gps.pData[(r * 4) + c]
#define K_baro(r, c)             _K_baro.pData[(r * 2) + c]
#define K_rangefinder(r, c)      _K_rangefinder.pData[(r * 2) + c]
#define V_accel(r, c)            _V_accel.pData[(r * 3) + c]
#define V_mag(r, c)              _V_mag.pData[(r * 3) + c]
#define V_gps(r, c)              _V_gps.pData[(r * 4) + c]
#define V_baro(r, c)             _V_baro.pData[(r * 2) + c]
#define V_rangefinder(r, c)      _V_rangefinder.pData[(r * 2) + c]
#define PHt_accel(r, c)          _PHt_accel.pData[(r * 3) + c]
#define PHt_mag(r, c)            _PHt_mag.pData[(r * 3) + c]
#define PHt_gps(r, c)            _PHt_gps.pData[(r * 4) + c]
#define PHt_baro(r, c)           _PHt_baro.pData[(r * 2) + c]
#define PHt_rangefinder(r, c)    _PHt_rangefinder.pData[(r * 2) + c]
#define HPHt_V_accel(r, c)       _HPHt_V_accel.pData[(r * 3) + c]
#define HPHt_V_mag(r, c)         _HPHt_V_mag.pData[(r * 3) + c]
#define HPHt_V_gps(r, c)         _HPHt_V_gps.pData[(r * 4) + c]
#define HPHt_V_baro(r, c)        _HPHt_V_baro.pData[(r * 2) + c]
#define HPHt_V_rangefinder(r, c) _HPHt_V_rangefinder.pData[(r * 2) + c]

ins_eskf_t ins_eskf;

MAT_ALLOC(nominal_state, 10, 1);
MAT_ALLOC(error_state, 9, 1);
MAT_ALLOC(_Q_i, 6, 6);
MAT_ALLOC(_V_accel, 3, 3);
MAT_ALLOC(_V_mag, 3, 3);
MAT_ALLOC(_V_gps, 4, 4);
MAT_ALLOC(_V_baro, 2, 2);
MAT_ALLOC(_V_rangefinder, 2, 2);
MAT_ALLOC(_P_prior, 9, 9);
MAT_ALLOC(_P_post, 9, 9);
MAT_ALLOC(_R, 3, 3);
MAT_ALLOC(_Rt, 3, 3);
MAT_ALLOC(_R_am_ab_dt, 3, 3);
MAT_ALLOC(_Rt_wm_wb_dt, 3, 3);
MAT_ALLOC(_PHt_accel, 9, 3);
MAT_ALLOC(_PHt_mag, 9, 3);
MAT_ALLOC(_PHt_gps, 9, 4);
MAT_ALLOC(_PHt_baro, 9, 2);
MAT_ALLOC(_PHt_rangefinder, 9, 2);
MAT_ALLOC(_HPHt_V_accel, 3, 3);
MAT_ALLOC(_HPHt_V_mag, 3, 3);
MAT_ALLOC(_HPHt_V_gps, 4, 4);
MAT_ALLOC(_HPHt_V_baro, 2, 2);
MAT_ALLOC(_HPHt_V_rangefinder, 2, 2);
MAT_ALLOC(_HPHt_V_accel_inv, 3, 3);
MAT_ALLOC(_HPHt_V_mag_inv, 3, 3);
MAT_ALLOC(_HPHt_V_gps_inv, 4, 4);
MAT_ALLOC(_HPHt_V_baro_inv, 2, 2);
MAT_ALLOC(_HPHt_V_rangefinder_inv, 2, 2);
MAT_ALLOC(_K_accel, 9, 3);
MAT_ALLOC(_K_mag, 9, 3);
MAT_ALLOC(_K_gps, 9, 4);
MAT_ALLOC(_K_baro, 9, 2);
MAT_ALLOC(_K_rangefinder, 9, 2);

float dt;
float half_dt;
float half_dt_squared;

void ins_eskf_reset_process_covariance_matrix(void)
{
	/* initialize P matrix */
	matrix_reset(mat_data(_P_post), 9, 9);
	P_post(0, 0) = 5.0f; //Var(px)
	P_post(1, 1) = 5.0f; //Var(py)
	P_post(2, 2) = 5.0f; //Var(pz)
	P_post(3, 3) = 5.0f; //Var(vx)
	P_post(4, 4) = 5.0f; //Var(vy)
	P_post(5, 5) = 5.0f; //Var(vz)
	P_post(6, 6) = 5.0f; //Var(theta_x)
	P_post(7, 7) = 5.0f; //Var(theta_y)
	P_post(8, 8) = 5.0f; //Var(theta_z)

	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);
}

void ins_eskf_init(float _dt)
{
	dt = _dt;
	half_dt = 0.5f * dt;
	half_dt_squared = 0.5f * dt * dt;

	MAT_INIT(nominal_state, 10, 1);
	MAT_INIT(error_state, 9, 1);
	MAT_INIT(_Q_i, 6, 6);
	MAT_INIT(_V_accel, 3, 3);
	MAT_INIT(_V_mag, 3, 3);
	MAT_INIT(_V_gps, 4, 4);
	MAT_INIT(_V_baro, 2, 2);
	MAT_INIT(_V_rangefinder, 2, 2);
	MAT_INIT(_P_prior, 9, 9);
	MAT_INIT(_P_post, 9, 9);
	MAT_INIT(_R, 3, 3);
	MAT_INIT(_Rt, 3, 3);
	MAT_INIT(_R_am_ab_dt, 3, 3);
	MAT_INIT(_Rt_wm_wb_dt, 3, 3);
	MAT_INIT(_PHt_accel, 9, 3);
	MAT_INIT(_PHt_mag, 9, 3);
	MAT_INIT(_PHt_gps, 9, 4);
	MAT_INIT(_PHt_baro, 9, 2);
	MAT_INIT(_PHt_rangefinder, 9, 2);
	MAT_INIT(_HPHt_V_accel, 3, 3);
	MAT_INIT(_HPHt_V_mag, 3, 3);
	MAT_INIT(_HPHt_V_gps, 4, 4);
	MAT_INIT(_HPHt_V_baro, 2, 2);
	MAT_INIT(_HPHt_V_rangefinder, 2, 2);
	MAT_INIT(_HPHt_V_accel_inv, 3, 3);
	MAT_INIT(_HPHt_V_mag_inv, 3, 3);
	MAT_INIT(_HPHt_V_gps_inv, 4, 4);
	MAT_INIT(_HPHt_V_baro_inv, 2, 2);
	MAT_INIT(_HPHt_V_rangefinder_inv, 2, 2);
	MAT_INIT(_K_accel, 9, 3);
	MAT_INIT(_K_mag, 9, 3);
	MAT_INIT(_K_gps, 9, 4);
	MAT_INIT(_K_baro, 9, 2);
	MAT_INIT(_K_rangefinder, 9, 2);

	/* initialize the nominal state */
	mat_data(nominal_state)[0] = 0.0f; //px
	mat_data(nominal_state)[1] = 0.0f; //py
	mat_data(nominal_state)[2] = 0.0f; //pz
	mat_data(nominal_state)[3] = 0.0f; //vx
	mat_data(nominal_state)[4] = 0.0f; //vy
	mat_data(nominal_state)[5] = 0.0f; //vz
	mat_data(nominal_state)[6] = 1.0f; //q0
	mat_data(nominal_state)[7] = 0.0f; //q1
	mat_data(nominal_state)[8] = 0.0f; //q2
	mat_data(nominal_state)[9] = 0.0f; //q3

	float q_i2b[4];
	init_ahrs_quaternion_with_accel_and_compass(q_i2b);
	quaternion_conj(q_i2b, &mat_data(nominal_state)[6]);

	matrix_reset(mat_data(error_state), 9, 1);

	/* initialize error-state process covariance matrix */
	ins_eskf_reset_process_covariance_matrix();

	/* initialize _Q_i matrix */
	matrix_reset(mat_data(_Q_i), 6, 6);
	Q_i(0, 0) = 1e-5; //Var(ax)
	Q_i(1, 1) = 1e-5; //Var(ay)
	Q_i(2, 2) = 1e-5; //Var(az)
	Q_i(3, 3) = 1e-5; //Var(wx)
	Q_i(4, 4) = 1e-5; //Var(wy)
	Q_i(5, 5) = 1e-5; //Var(wz)

	/* initialize V_accel matrix */
	matrix_reset(mat_data(_V_accel), 3, 3);
	V_accel(0, 0) = 7e-1; //Var(gx)
	V_accel(1, 1) = 7e-1; //Var(gy)
	V_accel(2, 2) = 7e-1; //Var(gz)

	/* initialize V_mag matrix */
	matrix_reset(mat_data(_V_mag), 3, 3);
	V_mag(0, 0) = 5e-1; //Var(mx)
	V_mag(1, 1) = 5e-1; //Var(my)
	V_mag(2, 2) = 5e-1; //Var(mz)

	/* initial V_gps matrix */
	matrix_reset(mat_data(_V_gps), 4, 4);
	V_gps(0, 0) = 1e-4; //Var(px)
	V_gps(1, 1) = 1e-4; //Var(py)
	V_gps(2, 2) = 1e-4; //Var(vx)
	V_gps(3, 3) = 1e-4; //Var(vy)

	/* initialize V_baro matrix */
	matrix_reset(mat_data(_V_baro), 2, 2);
	V_baro(0, 0) = 1e-1; //Var(pz)
	V_baro(1, 1) = 1e-1; //Var(vz)

	/* initialize V_rangefinder matrix */
	matrix_reset(mat_data(_V_rangefinder), 2, 2);
	V_rangefinder(0, 0) = 2.5e-2; //Var(pz)
	V_rangefinder(1, 1) = 2.5e-2; //Var(vz)

	matrix_reset(mat_data(_PHt_accel), 9, 3);
	matrix_reset(mat_data(_PHt_mag), 9, 3);
	matrix_reset(mat_data(_PHt_gps), 9, 4);
	matrix_reset(mat_data(_PHt_baro), 9, 2);
	matrix_reset(mat_data(_PHt_rangefinder), 9, 2);

	matrix_reset(mat_data(_HPHt_V_accel), 3, 3);
	matrix_reset(mat_data(_HPHt_V_mag), 3, 3);
	matrix_reset(mat_data(_HPHt_V_gps), 4, 4);
	matrix_reset(mat_data(_HPHt_V_baro), 2, 2);
	matrix_reset(mat_data(_HPHt_V_rangefinder), 2, 2);
}

float ins_eskf_get_covariance_matrix_norm(void)
{
	float norm = 0;

	int i;
	for(i = 0; i < 9; i++) {
		norm += P_prior(i, i);
	}

	return norm;
}

bool ins_eskf_sensor_all_ready(void)
{
	bool gps_ready = is_gps_available();
	bool compass_ready = is_compass_available();

#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	bool height_ready = is_barometer_available();
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	bool height_ready = rangefinder_available();
#else
	bool height_ready = false;
#endif

	return gps_ready && compass_ready && height_ready;
}

bool ins_eskf_is_stable(void)
{
	bool sensor_all_ready = ins_eskf_sensor_all_ready();
	bool eskf_converged = ins_eskf_get_covariance_matrix_norm() < ESKF_CONVERGENCE_NORM;

	if((sensor_all_ready == true) && (eskf_converged == true)) {
		return true;
	} else {
		return false;
	}
}

void ins_eskf_predict(float *accel, float *gyro)
{
	/* input variables (ned frame) */
	float accel_b_x = accel[0];
	float accel_b_y = accel[1];
	float accel_b_z = accel[2];
	float gyro_b_x = gyro[0];
	float gyro_b_y = gyro[1];
	float gyro_b_z = gyro[2];

	/* body-frame to inertial-frame conversion */
	float accel_i[3] = {0};
	accel_i[0] = R(0, 0) * accel_b_x + R(0, 1) * accel_b_y + R(0, 2) * accel_b_z;
	accel_i[1] = R(1, 0) * accel_b_x + R(1, 1) * accel_b_y + R(1, 2) * accel_b_z;
	accel_i[2] = R(2, 0) * accel_b_x + R(2, 1) * accel_b_y + R(2, 2) * accel_b_z;

	/* gravity compensation */
	accel_i[2] += 9.78f;

	/*======================*
	 * nominal state update *
	 *======================*/

	//velocity integration
	mat_data(nominal_state)[3] += accel_i[0] * dt;
	mat_data(nominal_state)[4] += accel_i[1] * dt;
	mat_data(nominal_state)[5] += accel_i[2] * dt;

	//position integration
	mat_data(nominal_state)[0] += (mat_data(nominal_state)[3] * dt) +
	                              (accel_i[0] * half_dt_squared);
	mat_data(nominal_state)[1] += (mat_data(nominal_state)[4] * dt) +
	                              (accel_i[1] * half_dt_squared);
	mat_data(nominal_state)[2] += (mat_data(nominal_state)[5] * dt) +
	                              (accel_i[2] * half_dt_squared);

	/* calculate quaternion time derivative */
	float w[4];
	w[0] = 0.0f;
	w[1] = gyro[0];
	w[2] = gyro[1];
	w[3] = gyro[2];
	float q_dot[4];
	quaternion_mult(&mat_data(nominal_state)[6], w, q_dot);

	//quaternion integration
	mat_data(nominal_state)[6] = mat_data(nominal_state)[6] + (q_dot[0] * half_dt);
	mat_data(nominal_state)[7] = mat_data(nominal_state)[7] + (q_dot[1] * half_dt);
	mat_data(nominal_state)[8] = mat_data(nominal_state)[8] + (q_dot[2] * half_dt);
	mat_data(nominal_state)[9] = mat_data(nominal_state)[9] + (q_dot[3] * half_dt);
	quat_normalize(&mat_data(nominal_state)[6]);

	/*==================================*
	 * process covatiance matrix update *
	 *==================================*/

	/* codeblock for preventing nameing conflict */
	{
		/* calculate -R * skew_matrix(a_m - a_b) * dt */
		R_am_ab_dt(0, 0) = -dt*(R(0,1)*accel_b_z-R(0,2)*accel_b_y);
		R_am_ab_dt(0, 1) = dt*(R(0,0)*accel_b_z-R(0,2)*accel_b_x);
		R_am_ab_dt(0, 2) = -dt*(R(0,0)*accel_b_y-R(0,1)*accel_b_x);
		R_am_ab_dt(1, 0) = -dt*(R(1,1)*accel_b_z-R(1,2)*accel_b_y);
		R_am_ab_dt(1, 1) = dt*(R(1,0)*accel_b_z-R(1,2)*accel_b_x);
		R_am_ab_dt(1, 2) = -dt*(R(1,0)*accel_b_y-R(1,1)*accel_b_x);
		R_am_ab_dt(2, 0) = -dt*(R(2,1)*accel_b_z-R(2,2)*accel_b_y);
		R_am_ab_dt(2, 1) = dt*(R(2,0)*accel_b_z-R(2,2)*accel_b_x);
		R_am_ab_dt(2, 2) = -dt*(R(2,0)*accel_b_y-R(2,1)*accel_b_x);

		/* calculate Rt{w_m - w_b} * dt */
		float c0 = dt*gyro_b_z;
		float c1 = dt*gyro_b_y;
		float c2 = dt*gyro_b_x;

		Rt_wm_wb_dt(0, 0) = 1.0;
		Rt_wm_wb_dt(0, 1) = c0;
		Rt_wm_wb_dt(0, 2) = -c1;
		Rt_wm_wb_dt(1, 0) = -c0;
		Rt_wm_wb_dt(1, 1) = 1.0;
		Rt_wm_wb_dt(1, 2) = c2;
		Rt_wm_wb_dt(2, 0) = c1;
		Rt_wm_wb_dt(2, 1) = -c2;
		Rt_wm_wb_dt(2, 2) = 1.0;
	}

	/* codeblock for preventing nameing conflict */
	{
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

	/*=================================================*
	 * convert estimated quaternion to R and Rt matrix *
	 *=================================================*/
	float *q = &mat_data(nominal_state)[6];
	quat_to_rotation_matrix(q, mat_data(_R), mat_data(_Rt));
}

void ins_eskf_accelerometer_correct(float *accel)
{
	float accel_sum_squared = (accel[0])*(accel[0]) + (accel[1])*(accel[1]) +
	                          (accel[2])*(accel[2]) + (accel[3])*(accel[3]);
	float div_accel_norm;
	arm_sqrt_f32(accel_sum_squared, &div_accel_norm);
	div_accel_norm = 1.0f / div_accel_norm;

	float gx = -accel[0] * div_accel_norm;
	float gy = -accel[1] * div_accel_norm;
	float gz = -accel[2] * div_accel_norm;

	float q0 = mat_data(nominal_state)[6];
	float q1 = mat_data(nominal_state)[7];
	float q2 = mat_data(nominal_state)[8];
	float q3 = mat_data(nominal_state)[9];

	/* codeblock for preventing nameing conflict */
	{
		/* calculate P * Ht */
		float c0 = q0*q0-q1*q1-q2*q2+q3*q3;
		float c1 = q0*q2*2.0-q1*q3*2.0;
		float c2 = q0*q1*2.0+q2*q3*2.0;

		PHt_accel(0, 0) = -P_prior(0,7)*c0+P_prior(0,8)*c2;
		PHt_accel(0, 1) = P_prior(0,6)*c0+P_prior(0,8)*c1;
		PHt_accel(0, 2) = -P_prior(0,6)*c2-P_prior(0,7)*c1;
		PHt_accel(1, 0) = -P_prior(1,7)*c0+P_prior(1,8)*c2;
		PHt_accel(1, 1) = P_prior(1,6)*c0+P_prior(1,8)*c1;
		PHt_accel(1, 2) = -P_prior(1,6)*c2-P_prior(1,7)*c1;
		PHt_accel(2, 0) = -P_prior(2,7)*c0+P_prior(2,8)*c2;
		PHt_accel(2, 1) = P_prior(2,6)*c0+P_prior(2,8)*c1;
		PHt_accel(2, 2) = -P_prior(2,6)*c2-P_prior(2,7)*c1;
		PHt_accel(3, 0) = -P_prior(3,7)*c0+P_prior(3,8)*c2;
		PHt_accel(3, 1) = P_prior(3,6)*c0+P_prior(3,8)*c1;
		PHt_accel(3, 2) = -P_prior(3,6)*c2-P_prior(3,7)*c1;
		PHt_accel(4, 0) = -P_prior(4,7)*c0+P_prior(4,8)*c2;
		PHt_accel(4, 1) = P_prior(4,6)*c0+P_prior(4,8)*c1;
		PHt_accel(4, 2) = -P_prior(4,6)*c2-P_prior(4,7)*c1;
		PHt_accel(5, 0) = -P_prior(5,7)*c0+P_prior(5,8)*c2;
		PHt_accel(5, 1) = P_prior(5,6)*c0+P_prior(5,8)*c1;
		PHt_accel(5, 2) = -P_prior(5,6)*c2-P_prior(5,7)*c1;
		PHt_accel(6, 0) = -P_prior(6,7)*c0+P_prior(6,8)*c2;
		PHt_accel(6, 1) = P_prior(6,6)*c0+P_prior(6,8)*c1;
		PHt_accel(6, 2) = -P_prior(6,6)*c2-P_prior(6,7)*c1;
		PHt_accel(7, 0) = -P_prior(7,7)*c0+P_prior(7,8)*c2;
		PHt_accel(7, 1) = P_prior(7,6)*c0+P_prior(7,8)*c1;
		PHt_accel(7, 2) = -P_prior(7,6)*c2-P_prior(7,7)*c1;
		PHt_accel(8, 0) = -P_prior(8,7)*c0+P_prior(8,8)*c2;
		PHt_accel(8, 1) = P_prior(8,6)*c0+P_prior(8,8)*c1;
		PHt_accel(8, 2) = -P_prior(8,6)*c2-P_prior(8,7)*c1;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate (H * P * Ht) + V */
		float c0 = q0*q0-q1*q1-q2*q2+q3*q3;
		float c1 = q0*q2*2.0-q1*q3*2.0;
		float c2 = q0*q1*2.0+q2*q3*2.0;

		HPHt_V_accel(0, 0) = V_accel(0,0)-PHt_accel(7,0)*c0+PHt_accel(8,0)*c2;
		HPHt_V_accel(0, 1) = -PHt_accel(7,1)*c0+PHt_accel(8,1)*c2;
		HPHt_V_accel(0, 2) = -PHt_accel(7,2)*c0+PHt_accel(8,2)*c2;
		HPHt_V_accel(1, 0) = PHt_accel(6,0)*c0+PHt_accel(8,0)*c1;
		HPHt_V_accel(1, 1) = V_accel(1,1)+PHt_accel(6,1)*c0+PHt_accel(8,1)*c1;
		HPHt_V_accel(1, 2) = PHt_accel(6,2)*c0+PHt_accel(8,2)*c1;
		HPHt_V_accel(2, 0) = -PHt_accel(6,0)*c2-PHt_accel(7,0)*c1;
		HPHt_V_accel(2, 1) = -PHt_accel(6,1)*c2-PHt_accel(7,1)*c1;
		HPHt_V_accel(2, 2) = V_accel(2,2)-PHt_accel(6,2)*c2-PHt_accel(7,2)*c1;
	}

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_INV(&_HPHt_V_accel, &_HPHt_V_accel_inv);
	MAT_MULT(&_PHt_accel, &_HPHt_V_accel_inv, &_K_accel);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate error state residual */
		float c0 = gz-q0*q0+q1*q1+q2*q2-q3*q3;
		float c1 = -gy+q0*q1*2.0+q2*q3*2.0;
		float c2 = gx+q0*q2*2.0-q1*q3*2.0;

		mat_data(error_state)[0] = K_accel(0,0)*c2-K_accel(0,1)*c1+K_accel(0,2)*c0;
		mat_data(error_state)[1] = K_accel(1,0)*c2-K_accel(1,1)*c1+K_accel(1,2)*c0;
		mat_data(error_state)[2] = K_accel(2,0)*c2-K_accel(2,1)*c1+K_accel(2,2)*c0;
		mat_data(error_state)[3] = K_accel(3,0)*c2-K_accel(3,1)*c1+K_accel(3,2)*c0;
		mat_data(error_state)[4] = K_accel(4,0)*c2-K_accel(4,1)*c1+K_accel(4,2)*c0;
		mat_data(error_state)[5] = K_accel(5,0)*c2-K_accel(5,1)*c1+K_accel(5,2)*c0;
		mat_data(error_state)[6] = K_accel(6,0)*c2-K_accel(6,1)*c1+K_accel(6,2)*c0;
		mat_data(error_state)[7] = K_accel(7,0)*c2-K_accel(7,1)*c1+K_accel(7,2)*c0;
		mat_data(error_state)[8] = K_accel(8,0)*c2-K_accel(8,1)*c1+K_accel(8,2)*c0;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate a posteriori process covariance matrix */
		//P = (I - K*H) * P
		float c0 = q0*q0-q1*q1-q2*q2+q3*q3;
		float c1 = q0*q2*2.0-q1*q3*2.0;
		float c2 = q0*q1*2.0+q2*q3*2.0;
		float c3 = -K_accel(6,1)*c0+K_accel(6,2)*c2+1.0;
		float c4 = K_accel(8,1)*c0-K_accel(8,2)*c2;
		float c5 = K_accel(7,1)*c0-K_accel(7,2)*c2;
		float c6 = K_accel(5,1)*c0-K_accel(5,2)*c2;
		float c7 = K_accel(4,1)*c0-K_accel(4,2)*c2;
		float c8 = K_accel(3,1)*c0-K_accel(3,2)*c2;
		float c9 = K_accel(2,1)*c0-K_accel(2,2)*c2;
		float c10 = K_accel(1,1)*c0-K_accel(1,2)*c2;
		float c11 = K_accel(0,1)*c0-K_accel(0,2)*c2;
		float c12 = K_accel(8,0)*c2+K_accel(8,1)*c1-1.0;
		float c13 = K_accel(7,0)*c0+K_accel(7,2)*c1+1.0;
		float c14 = K_accel(8,0)*c0+K_accel(8,2)*c1;
		float c15 = K_accel(7,0)*c2+K_accel(7,1)*c1;
		float c16 = K_accel(6,0)*c2+K_accel(6,1)*c1;
		float c17 = K_accel(6,0)*c0+K_accel(6,2)*c1;
		float c18 = K_accel(5,0)*c2+K_accel(5,1)*c1;
		float c19 = K_accel(5,0)*c0+K_accel(5,2)*c1;
		float c20 = K_accel(4,0)*c2+K_accel(4,1)*c1;
		float c21 = K_accel(4,0)*c0+K_accel(4,2)*c1;
		float c22 = K_accel(3,0)*c2+K_accel(3,1)*c1;
		float c23 = K_accel(3,0)*c0+K_accel(3,2)*c1;
		float c24 = K_accel(2,0)*c2+K_accel(2,1)*c1;
		float c25 = K_accel(2,0)*c0+K_accel(2,2)*c1;
		float c26 = K_accel(1,0)*c2+K_accel(1,1)*c1;
		float c27 = K_accel(1,0)*c0+K_accel(1,2)*c1;
		float c28 = K_accel(0,0)*c2+K_accel(0,1)*c1;
		float c29 = K_accel(0,0)*c0+K_accel(0,2)*c1;

		P_post(0, 0) = P_prior(0,0)-P_prior(6,0)*c11+P_prior(7,0)*c29-P_prior(8,0)*c28;
		P_post(0, 1) = P_prior(0,1)-P_prior(6,1)*c11+P_prior(7,1)*c29-P_prior(8,1)*c28;
		P_post(0, 2) = P_prior(0,2)-P_prior(6,2)*c11+P_prior(7,2)*c29-P_prior(8,2)*c28;
		P_post(0, 3) = P_prior(0,3)-P_prior(6,3)*c11+P_prior(7,3)*c29-P_prior(8,3)*c28;
		P_post(0, 4) = P_prior(0,4)-P_prior(6,4)*c11+P_prior(7,4)*c29-P_prior(8,4)*c28;
		P_post(0, 5) = P_prior(0,5)-P_prior(6,5)*c11+P_prior(7,5)*c29-P_prior(8,5)*c28;
		P_post(0, 6) = P_prior(0,6)-P_prior(6,6)*c11+P_prior(7,6)*c29-P_prior(8,6)*c28;
		P_post(0, 7) = P_prior(0,7)-P_prior(6,7)*c11+P_prior(7,7)*c29-P_prior(8,7)*c28;
		P_post(0, 8) = P_prior(0,8)-P_prior(6,8)*c11+P_prior(7,8)*c29-P_prior(8,8)*c28;
		P_post(1, 0) = P_post(0, 1);
		P_post(1, 1) = P_prior(1,1)-P_prior(6,1)*c10+P_prior(7,1)*c27-P_prior(8,1)*c26;
		P_post(1, 2) = P_prior(1,2)-P_prior(6,2)*c10+P_prior(7,2)*c27-P_prior(8,2)*c26;
		P_post(1, 3) = P_prior(1,3)-P_prior(6,3)*c10+P_prior(7,3)*c27-P_prior(8,3)*c26;
		P_post(1, 4) = P_prior(1,4)-P_prior(6,4)*c10+P_prior(7,4)*c27-P_prior(8,4)*c26;
		P_post(1, 5) = P_prior(1,5)-P_prior(6,5)*c10+P_prior(7,5)*c27-P_prior(8,5)*c26;
		P_post(1, 6) = P_prior(1,6)-P_prior(6,6)*c10+P_prior(7,6)*c27-P_prior(8,6)*c26;
		P_post(1, 7) = P_prior(1,7)-P_prior(6,7)*c10+P_prior(7,7)*c27-P_prior(8,7)*c26;
		P_post(1, 8) = P_prior(1,8)-P_prior(6,8)*c10+P_prior(7,8)*c27-P_prior(8,8)*c26;
		P_post(2, 0) = P_post(0, 2);
		P_post(2, 1) = P_post(1, 2);
		P_post(2, 2) = P_prior(2,2)-P_prior(6,2)*c9+P_prior(7,2)*c25-P_prior(8,2)*c24;
		P_post(2, 3) = P_prior(2,3)-P_prior(6,3)*c9+P_prior(7,3)*c25-P_prior(8,3)*c24;
		P_post(2, 4) = P_prior(2,4)-P_prior(6,4)*c9+P_prior(7,4)*c25-P_prior(8,4)*c24;
		P_post(2, 5) = P_prior(2,5)-P_prior(6,5)*c9+P_prior(7,5)*c25-P_prior(8,5)*c24;
		P_post(2, 6) = P_prior(2,6)-P_prior(6,6)*c9+P_prior(7,6)*c25-P_prior(8,6)*c24;
		P_post(2, 7) = P_prior(2,7)-P_prior(6,7)*c9+P_prior(7,7)*c25-P_prior(8,7)*c24;
		P_post(2, 8) = P_prior(2,8)-P_prior(6,8)*c9+P_prior(7,8)*c25-P_prior(8,8)*c24;
		P_post(3, 0) = P_post(0, 3);
		P_post(3, 1) = P_post(1, 3);
		P_post(3, 2) = P_post(2, 3);
		P_post(3, 3) = P_prior(3,3)-P_prior(6,3)*c8+P_prior(7,3)*c23-P_prior(8,3)*c22;
		P_post(3, 4) = P_prior(3,4)-P_prior(6,4)*c8+P_prior(7,4)*c23-P_prior(8,4)*c22;
		P_post(3, 5) = P_prior(3,5)-P_prior(6,5)*c8+P_prior(7,5)*c23-P_prior(8,5)*c22;
		P_post(3, 6) = P_prior(3,6)-P_prior(6,6)*c8+P_prior(7,6)*c23-P_prior(8,6)*c22;
		P_post(3, 7) = P_prior(3,7)-P_prior(6,7)*c8+P_prior(7,7)*c23-P_prior(8,7)*c22;
		P_post(3, 8) = P_prior(3,8)-P_prior(6,8)*c8+P_prior(7,8)*c23-P_prior(8,8)*c22;
		P_post(4, 0) = P_post(0, 4);
		P_post(4, 1) = P_post(1, 4);
		P_post(4, 2) = P_post(2, 4);
		P_post(4, 3) = P_post(3, 4);
		P_post(4, 4) = P_prior(4,4)-P_prior(6,4)*c7+P_prior(7,4)*c21-P_prior(8,4)*c20;
		P_post(4, 5) = P_prior(4,5)-P_prior(6,5)*c7+P_prior(7,5)*c21-P_prior(8,5)*c20;
		P_post(4, 6) = P_prior(4,6)-P_prior(6,6)*c7+P_prior(7,6)*c21-P_prior(8,6)*c20;
		P_post(4, 7) = P_prior(4,7)-P_prior(6,7)*c7+P_prior(7,7)*c21-P_prior(8,7)*c20;
		P_post(4, 8) = P_prior(4,8)-P_prior(6,8)*c7+P_prior(7,8)*c21-P_prior(8,8)*c20;
		P_post(5, 0) = P_post(0, 5);
		P_post(5, 1) = P_post(1, 5);
		P_post(5, 2) = P_post(2, 5);
		P_post(5, 3) = P_post(3, 5);
		P_post(5, 4) = P_post(4, 5);
		P_post(5, 5) = P_prior(5,5)-P_prior(6,5)*c6+P_prior(7,5)*c19-P_prior(8,5)*c18;
		P_post(5, 6) = P_prior(5,6)-P_prior(6,6)*c6+P_prior(7,6)*c19-P_prior(8,6)*c18;
		P_post(5, 7) = P_prior(5,7)-P_prior(6,7)*c6+P_prior(7,7)*c19-P_prior(8,7)*c18;
		P_post(5, 8) = P_prior(5,8)-P_prior(6,8)*c6+P_prior(7,8)*c19-P_prior(8,8)*c18;
		P_post(6, 0) = P_post(0, 6);
		P_post(6, 1) = P_post(1, 6);
		P_post(6, 2) = P_post(2, 6);
		P_post(6, 3) = P_post(3, 6);
		P_post(6, 4) = P_post(4, 6);
		P_post(6, 5) = P_post(5, 6);
		P_post(6, 6) = P_prior(6,6)*c3+P_prior(7,6)*c17-P_prior(8,6)*c16;
		P_post(6, 7) = P_prior(6,7)*c3+P_prior(7,7)*c17-P_prior(8,7)*c16;
		P_post(6, 8) = P_prior(6,8)*c3+P_prior(7,8)*c17-P_prior(8,8)*c16;
		P_post(7, 0) = P_post(0, 7);
		P_post(7, 1) = P_post(1, 7);
		P_post(7, 2) = P_post(2, 7);
		P_post(7, 3) = P_post(3, 7);
		P_post(7, 4) = P_post(4, 7);
		P_post(7, 5) = P_post(5, 7);
		P_post(7, 6) = P_post(6, 7);
		P_post(7, 7) = -P_prior(6,7)*c5+P_prior(7,7)*c13-P_prior(8,7)*c15;
		P_post(7, 8) = -P_prior(6,8)*c5+P_prior(7,8)*c13-P_prior(8,8)*c15;
		P_post(8, 0) = P_post(0, 8);
		P_post(8, 1) = P_post(1, 8);
		P_post(8, 2) = P_post(2, 8);
		P_post(8, 3) = P_post(3, 8);
		P_post(8, 4) = P_post(4, 8);
		P_post(8, 5) = P_post(5, 8);
		P_post(8, 6) = P_post(6, 8);
		P_post(8, 7) = P_post(7, 8);
		P_post(8, 8) = -P_prior(6,8)*c4+P_prior(7,8)*c14-P_prior(8,8)*c12;
	}

	/* P_post becoms the P_prior of other measurement's correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);

	/* error state injection */
	float q_error[4];
	q_error[0] = 1.0f;
	q_error[1] = 0.5 * mat_data(error_state)[6];
	q_error[2] = 0.5 * mat_data(error_state)[7];
	q_error[3] = 0.5 * mat_data(error_state)[8];

	//nominal_state (a posteriori) = q_error * nominal_state (a priori)
	float q_last[4];
	quaternion_copy(q_last, &mat_data(nominal_state)[6]);
	quaternion_mult(q_last, q_error, &mat_data(nominal_state)[6]);

	//renormailization
	quat_normalize(&mat_data(nominal_state)[6]);

	/*=================================================*
	 * convert estimated quaternion to R and Rt matrix *
	 *=================================================*/
	float *q = &mat_data(nominal_state)[6];
	quat_to_rotation_matrix(q, mat_data(_R), mat_data(_Rt));
}

void ins_eskf_magnetometer_correct(float *mag)
{
	float mag_sum_squared = (mag[0])*(mag[0]) + (mag[1])*(mag[1]) +
	                        (mag[2])*(mag[2]) + (mag[3])*(mag[3]);
	float div_mag_norm;
	arm_sqrt_f32(mag_sum_squared, &div_mag_norm);
	div_mag_norm = 1.0f / div_mag_norm;

	float mx = mag[0] * div_mag_norm;
	float my = mag[1] * div_mag_norm;
	float mz = mag[2] * div_mag_norm;

	float q0 = mat_data(nominal_state)[6];
	float q1 = mat_data(nominal_state)[7];
	float q2 = mat_data(nominal_state)[8];
	float q3 = mat_data(nominal_state)[9];

	float gamma = sqrt(mx*mx + my*my);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate P * Ht */
		float c0_ = gamma*q0*2.0-mz*q2*2.0;
		float c1_ = gamma*q1*2.0+mz*q3*2.0;
		float c2_ = gamma*q2*2.0+mz*q0*2.0;
		float c3_ = gamma*q3*2.0-mz*q1*2.0;

		float c0 = c0_*q3*(-1.0*0.5)+(c1_*q2)*0.5+(c2_*q1)*0.5-(c3_*q0)*0.5;
		float c1 = (c0_*q2)*0.5+(c2_*q0)*0.5+(c1_*q3)*0.5+(c3_*q1)*0.5;
		float c2 = c3_;
		float c3 = (c2*q2)*0.5-(c0_*q1)*0.5+(c1_*q0)*0.5-(c2_*q3)*0.5;
		float c4 = (c2*q3)*0.5-(c0_*q0)*0.5-(c1_*q1)*0.5+(c2_*q2)*0.5;

		PHt_mag(0, 0) = -P_prior(0,7)*c1+P_prior(0,8)*c0+P_prior(0,6)*c3;
		PHt_mag(0, 1) = P_prior(0,6)*c1+P_prior(0,7)*c3+P_prior(0,8)*c4;
		PHt_mag(0, 2) = -P_prior(0,6)*c0-P_prior(0,7)*c4+P_prior(0,8)*c3;
		PHt_mag(1, 0) = -P_prior(1,7)*c1+P_prior(1,8)*c0+P_prior(1,6)*c3;
		PHt_mag(1, 1) = P_prior(1,6)*c1+P_prior(1,7)*c3+P_prior(1,8)*c4;
		PHt_mag(1, 2) = -P_prior(1,6)*c0-P_prior(1,7)*c4+P_prior(1,8)*c3;
		PHt_mag(2, 0) = -P_prior(2,7)*c1+P_prior(2,8)*c0+P_prior(2,6)*c3;
		PHt_mag(2, 1) = P_prior(2,6)*c1+P_prior(2,7)*c3+P_prior(2,8)*c4;
		PHt_mag(2, 2) = -P_prior(2,6)*c0-P_prior(2,7)*c4+P_prior(2,8)*c3;
		PHt_mag(3, 0) = -P_prior(3,7)*c1+P_prior(3,8)*c0+P_prior(3,6)*c3;
		PHt_mag(3, 1) = P_prior(3,6)*c1+P_prior(3,7)*c3+P_prior(3,8)*c4;
		PHt_mag(3, 2) = -P_prior(3,6)*c0-P_prior(3,7)*c4+P_prior(3,8)*c3;
		PHt_mag(4, 0) = -P_prior(4,7)*c1+P_prior(4,8)*c0+P_prior(4,6)*c3;
		PHt_mag(4, 1) = P_prior(4,6)*c1+P_prior(4,7)*c3+P_prior(4,8)*c4;
		PHt_mag(4, 2) = -P_prior(4,6)*c0-P_prior(4,7)*c4+P_prior(4,8)*c3;
		PHt_mag(5, 0) = -P_prior(5,7)*c1+P_prior(5,8)*c0+P_prior(5,6)*c3;
		PHt_mag(5, 1) = P_prior(5,6)*c1+P_prior(5,7)*c3+P_prior(5,8)*c4;
		PHt_mag(5, 2) = -P_prior(5,6)*c0-P_prior(5,7)*c4+P_prior(5,8)*c3;
		PHt_mag(6, 0) = -P_prior(6,7)*c1+P_prior(6,8)*c0+P_prior(6,6)*c3;
		PHt_mag(6, 1) = P_prior(6,6)*c1+P_prior(6,7)*c3+P_prior(6,8)*c4;
		PHt_mag(6, 2) = -P_prior(6,6)*c0-P_prior(6,7)*c4+P_prior(6,8)*c3;
		PHt_mag(7, 0) = -P_prior(7,7)*c1+P_prior(7,8)*c0+P_prior(7,6)*c3;
		PHt_mag(7, 1) = P_prior(7,6)*c1+P_prior(7,7)*c3+P_prior(7,8)*c4;
		PHt_mag(7, 2) = -P_prior(7,6)*c0-P_prior(7,7)*c4+P_prior(7,8)*c3;
		PHt_mag(8, 0) = -P_prior(8,7)*c1+P_prior(8,8)*c0+P_prior(8,6)*c3;
		PHt_mag(8, 1) = P_prior(8,6)*c1+P_prior(8,7)*c3+P_prior(8,8)*c4;
		PHt_mag(8, 2) = -P_prior(8,6)*c0-P_prior(8,7)*c4+P_prior(8,8)*c3;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate (H * P * Ht) + V */
		float c0_ = gamma*q0*2.0-mz*q2*2.0;
		float c1_ = gamma*q1*2.0+mz*q3*2.0;
		float c2_ = gamma*q2*2.0+mz*q0*2.0;
		float c3_ = gamma*q3*2.0-mz*q1*2.0;

		float c0 = c0_*q3*(-1.0*0.5)+(c1_*q2)*0.5+(c2_*q1)*0.5-(c3_*q0)*0.5;
		float c1 = (c0_*q2)*0.5+(c2_*q0)*0.5+(c1_*q3)*0.5+(c3_*q1)*0.5;
		float c2 = c3_;
		float c3 = (c2*q2)*0.5-(c0_*q1)*0.5+(c1_*q0)*0.5-(c2_*q3)*0.5;
		float c4 = (c2*q3)*0.5-(c0_*q0)*0.5-(c1_*q1)*0.5+(c2_*q2)*0.5;

		HPHt_V_mag(0, 0) = V_mag(0,0)+PHt_mag(6,0)*c3-PHt_mag(7,0)*c1+PHt_mag(8,0)*c0;
		HPHt_V_mag(0, 1) = PHt_mag(6,1)*c3-PHt_mag(7,1)*c1+PHt_mag(8,1)*c0;
		HPHt_V_mag(0, 2) = PHt_mag(6,2)*c3-PHt_mag(7,2)*c1+PHt_mag(8,2)*c0;
		HPHt_V_mag(1, 0) = PHt_mag(6,0)*c1+PHt_mag(7,0)*c3+PHt_mag(8,0)*c4;
		HPHt_V_mag(1, 1) = V_mag(1,1)+PHt_mag(6,1)*c1+PHt_mag(7,1)*c3+PHt_mag(8,1)*c4;
		HPHt_V_mag(1, 2) = PHt_mag(6,2)*c1+PHt_mag(7,2)*c3+PHt_mag(8,2)*c4;
		HPHt_V_mag(2, 0) = -PHt_mag(6,0)*c0-PHt_mag(7,0)*c4+PHt_mag(8,0)*c3;
		HPHt_V_mag(2, 1) = -PHt_mag(6,1)*c0-PHt_mag(7,1)*c4+PHt_mag(8,1)*c3;
		HPHt_V_mag(2, 2) = V_mag(2,2)-PHt_mag(6,2)*c0-PHt_mag(7,2)*c4+PHt_mag(8,2)*c3;
	}

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_INV(&_HPHt_V_mag, &_HPHt_V_mag_inv);
	MAT_MULT(&_PHt_mag, &_HPHt_V_mag_inv, &_K_mag);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate error state residual */
		float c0_ = -q2*q2;
		float c1_ = q3*q3;
		float c2_ = q1*q1;
		float c3_ = q0*q0;
		float c4_ = q0*q2;
		float c5_ = q1*q3;

		float c0 = mx-gamma*(c0_-c1_+c2_+c3_)+mz*(c4_-c5_)*2.0;
		float c1 = -mz+mz*(c0_+c1_-c2_+c3_)+gamma*(c4_+c5_)*2.0;
		float c2 = my+gamma*(q0*q3-q1*q2)*2.0-mz*(q0*q1+q2*q3)*2.0;

		mat_data(error_state)[0] = K_mag(0,0)*c0+K_mag(0,1)*c2-K_mag(0,2)*c1;
		mat_data(error_state)[1] = K_mag(1,0)*c0+K_mag(1,1)*c2-K_mag(1,2)*c1;
		mat_data(error_state)[2] = K_mag(2,0)*c0+K_mag(2,1)*c2-K_mag(2,2)*c1;
		mat_data(error_state)[3] = K_mag(3,0)*c0+K_mag(3,1)*c2-K_mag(3,2)*c1;
		mat_data(error_state)[4] = K_mag(4,0)*c0+K_mag(4,1)*c2-K_mag(4,2)*c1;
		mat_data(error_state)[5] = K_mag(5,0)*c0+K_mag(5,1)*c2-K_mag(5,2)*c1;
		mat_data(error_state)[6] = K_mag(6,0)*c0+K_mag(6,1)*c2-K_mag(6,2)*c1;
		mat_data(error_state)[7] = K_mag(7,0)*c0+K_mag(7,1)*c2-K_mag(7,2)*c1;
		mat_data(error_state)[8] = K_mag(8,0)*c0+K_mag(8,1)*c2-K_mag(8,2)*c1;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate a posteriori process covariance matrix */
		//P = (I - K*H) * P
		float c0_ = gamma*q3*2.0-mz*q1*2.0;
		float c1_ = gamma*q1*2.0+mz*q3*2.0;
		float c2_ = gamma*q2*2.0+mz*q0*2.0;
		float c3_ = gamma*q0*2.0-mz*q2*2.0;

		float c0 = c0_*q0*(-1.0*0.5)+(c1_*q2)*0.5+(c2_*q1)*0.5-(c3_*q3)*0.5;
		float c1 = (c0_*q1)*0.5+(c2_*q0)*0.5+(c1_*q3)*0.5+(c3_*q2)*0.5;
		float c2 = c3_;
		float c3 = (c2*q1)*0.5-(c1_*q0)*0.5-(c0_*q2)*0.5+(c2_*q3)*0.5;
		float c4 = (c2*q0)*0.5+(c1_*q1)*0.5-(c0_*q3)*0.5-(c2_*q2)*0.5;
		float c5 = -K_mag(8,0)*c0+K_mag(8,1)*c4+K_mag(8,2)*c3+1.0;
		float c6 = K_mag(7,0)*c1+K_mag(7,1)*c3-K_mag(7,2)*c4+1.0;
		float c7 = -K_mag(6,1)*c1+K_mag(6,2)*c0+K_mag(6,0)*c3+1.0;
		float c8 = K_mag(8,0)*c1+K_mag(8,1)*c3-K_mag(8,2)*c4;
		float c9 = -K_mag(8,1)*c1+K_mag(8,2)*c0+K_mag(8,0)*c3;
		float c10 = -K_mag(7,0)*c0+K_mag(7,1)*c4+K_mag(7,2)*c3;
		float c11 = -K_mag(7,1)*c1+K_mag(7,2)*c0+K_mag(7,0)*c3;
		float c12 = K_mag(6,0)*c1+K_mag(6,1)*c3-K_mag(6,2)*c4;
		float c13 = -K_mag(6,0)*c0+K_mag(6,1)*c4+K_mag(6,2)*c3;
		float c14 = K_mag(5,0)*c1+K_mag(5,1)*c3-K_mag(5,2)*c4;
		float c15 = -K_mag(5,0)*c0+K_mag(5,1)*c4+K_mag(5,2)*c3;
		float c16 = -K_mag(5,1)*c1+K_mag(5,2)*c0+K_mag(5,0)*c3;
		float c17 = K_mag(4,0)*c1+K_mag(4,1)*c3-K_mag(4,2)*c4;
		float c18 = -K_mag(4,0)*c0+K_mag(4,1)*c4+K_mag(4,2)*c3;
		float c19 = -K_mag(4,1)*c1+K_mag(4,2)*c0+K_mag(4,0)*c3;
		float c20 = K_mag(3,0)*c1+K_mag(3,1)*c3-K_mag(3,2)*c4;
		float c21 = -K_mag(3,0)*c0+K_mag(3,1)*c4+K_mag(3,2)*c3;
		float c22 = -K_mag(3,1)*c1+K_mag(3,2)*c0+K_mag(3,0)*c3;
		float c23 = K_mag(2,0)*c1+K_mag(2,1)*c3-K_mag(2,2)*c4;
		float c24 = -K_mag(2,0)*c0+K_mag(2,1)*c4+K_mag(2,2)*c3;
		float c25 = -K_mag(2,1)*c1+K_mag(2,2)*c0+K_mag(2,0)*c3;
		float c26 = K_mag(1,0)*c1+K_mag(1,1)*c3-K_mag(1,2)*c4;
		float c27 = -K_mag(1,0)*c0+K_mag(1,1)*c4+K_mag(1,2)*c3;
		float c28 = -K_mag(1,1)*c1+K_mag(1,2)*c0+K_mag(1,0)*c3;
		float c29 = K_mag(0,0)*c1+K_mag(0,1)*c3-K_mag(0,2)*c4;
		float c30 = -K_mag(0,0)*c0+K_mag(0,1)*c4+K_mag(0,2)*c3;
		float c31 = -K_mag(0,1)*c1+K_mag(0,2)*c0+K_mag(0,0)*c3;

		P_post(0, 0) = P_prior(0,0)+P_prior(6,0)*c31+P_prior(7,0)*c29+P_prior(8,0)*c30;
		P_post(0, 1) = P_prior(0,1)+P_prior(6,1)*c31+P_prior(7,1)*c29+P_prior(8,1)*c30;
		P_post(0, 2) = P_prior(0,2)+P_prior(6,2)*c31+P_prior(7,2)*c29+P_prior(8,2)*c30;
		P_post(0, 3) = P_prior(0,3)+P_prior(6,3)*c31+P_prior(7,3)*c29+P_prior(8,3)*c30;
		P_post(0, 4) = P_prior(0,4)+P_prior(6,4)*c31+P_prior(7,4)*c29+P_prior(8,4)*c30;
		P_post(0, 5) = P_prior(0,5)+P_prior(6,5)*c31+P_prior(7,5)*c29+P_prior(8,5)*c30;
		P_post(0, 6) = P_prior(0,6)+P_prior(6,6)*c31+P_prior(7,6)*c29+P_prior(8,6)*c30;
		P_post(0, 7) = P_prior(0,7)+P_prior(6,7)*c31+P_prior(7,7)*c29+P_prior(8,7)*c30;
		P_post(0, 8) = P_prior(0,8)+P_prior(6,8)*c31+P_prior(7,8)*c29+P_prior(8,8)*c30;
		P_post(1, 0) = P_post(0, 1);
		P_post(1, 1) = P_prior(1,1)+P_prior(6,1)*c28+P_prior(7,1)*c26+P_prior(8,1)*c27;
		P_post(1, 2) = P_prior(1,2)+P_prior(6,2)*c28+P_prior(7,2)*c26+P_prior(8,2)*c27;
		P_post(1, 3) = P_prior(1,3)+P_prior(6,3)*c28+P_prior(7,3)*c26+P_prior(8,3)*c27;
		P_post(1, 4) = P_prior(1,4)+P_prior(6,4)*c28+P_prior(7,4)*c26+P_prior(8,4)*c27;
		P_post(1, 5) = P_prior(1,5)+P_prior(6,5)*c28+P_prior(7,5)*c26+P_prior(8,5)*c27;
		P_post(1, 6) = P_prior(1,6)+P_prior(6,6)*c28+P_prior(7,6)*c26+P_prior(8,6)*c27;
		P_post(1, 7) = P_prior(1,7)+P_prior(6,7)*c28+P_prior(7,7)*c26+P_prior(8,7)*c27;
		P_post(1, 8) = P_prior(1,8)+P_prior(6,8)*c28+P_prior(7,8)*c26+P_prior(8,8)*c27;
		P_post(2, 0) = P_post(0, 2);
		P_post(2, 1) = P_post(1, 2);
		P_post(2, 2) = P_prior(2,2)+P_prior(6,2)*c25+P_prior(7,2)*c23+P_prior(8,2)*c24;
		P_post(2, 3) = P_prior(2,3)+P_prior(6,3)*c25+P_prior(7,3)*c23+P_prior(8,3)*c24;
		P_post(2, 4) = P_prior(2,4)+P_prior(6,4)*c25+P_prior(7,4)*c23+P_prior(8,4)*c24;
		P_post(2, 5) = P_prior(2,5)+P_prior(6,5)*c25+P_prior(7,5)*c23+P_prior(8,5)*c24;
		P_post(2, 6) = P_prior(2,6)+P_prior(6,6)*c25+P_prior(7,6)*c23+P_prior(8,6)*c24;
		P_post(2, 7) = P_prior(2,7)+P_prior(6,7)*c25+P_prior(7,7)*c23+P_prior(8,7)*c24;
		P_post(2, 8) = P_prior(2,8)+P_prior(6,8)*c25+P_prior(7,8)*c23+P_prior(8,8)*c24;
		P_post(3, 0) = P_post(0, 3);
		P_post(3, 1) = P_post(1, 3);
		P_post(3, 2) = P_post(2, 3);
		P_post(3, 3) = P_prior(3,3)+P_prior(6,3)*c22+P_prior(7,3)*c20+P_prior(8,3)*c21;
		P_post(3, 4) = P_prior(3,4)+P_prior(6,4)*c22+P_prior(7,4)*c20+P_prior(8,4)*c21;
		P_post(3, 5) = P_prior(3,5)+P_prior(6,5)*c22+P_prior(7,5)*c20+P_prior(8,5)*c21;
		P_post(3, 6) = P_prior(3,6)+P_prior(6,6)*c22+P_prior(7,6)*c20+P_prior(8,6)*c21;
		P_post(3, 7) = P_prior(3,7)+P_prior(6,7)*c22+P_prior(7,7)*c20+P_prior(8,7)*c21;
		P_post(3, 8) = P_prior(3,8)+P_prior(6,8)*c22+P_prior(7,8)*c20+P_prior(8,8)*c21;
		P_post(4, 0) = P_post(0, 4);
		P_post(4, 1) = P_post(1, 4);
		P_post(4, 2) = P_post(2, 4);
		P_post(4, 3) = P_post(3, 4);
		P_post(4, 4) = P_prior(4,4)+P_prior(6,4)*c19+P_prior(7,4)*c17+P_prior(8,4)*c18;
		P_post(4, 5) = P_prior(4,5)+P_prior(6,5)*c19+P_prior(7,5)*c17+P_prior(8,5)*c18;
		P_post(4, 6) = P_prior(4,6)+P_prior(6,6)*c19+P_prior(7,6)*c17+P_prior(8,6)*c18;
		P_post(4, 7) = P_prior(4,7)+P_prior(6,7)*c19+P_prior(7,7)*c17+P_prior(8,7)*c18;
		P_post(4, 8) = P_prior(4,8)+P_prior(6,8)*c19+P_prior(7,8)*c17+P_prior(8,8)*c18;
		P_post(5, 0) = P_post(0, 5);
		P_post(5, 1) = P_post(1, 5);
		P_post(5, 2) = P_post(2, 5);
		P_post(5, 3) = P_post(3, 5);
		P_post(5, 4) = P_post(4, 5);
		P_post(5, 5) = P_prior(5,5)+P_prior(6,5)*c16+P_prior(7,5)*c14+P_prior(8,5)*c15;
		P_post(5, 6) = P_prior(5,6)+P_prior(6,6)*c16+P_prior(7,6)*c14+P_prior(8,6)*c15;
		P_post(5, 7) = P_prior(5,7)+P_prior(6,7)*c16+P_prior(7,7)*c14+P_prior(8,7)*c15;
		P_post(5, 8) = P_prior(5,8)+P_prior(6,8)*c16+P_prior(7,8)*c14+P_prior(8,8)*c15;
		P_post(6, 0) = P_post(0, 6);
		P_post(6, 1) = P_post(1, 6);
		P_post(6, 2) = P_post(2, 6);
		P_post(6, 3) = P_post(3, 6);
		P_post(6, 4) = P_post(4, 6);
		P_post(6, 5) = P_post(5, 6);
		P_post(6, 6) = P_prior(6,6)*c7+P_prior(7,6)*c12+P_prior(8,6)*c13;
		P_post(6, 7) = P_prior(6,7)*c7+P_prior(7,7)*c12+P_prior(8,7)*c13;
		P_post(6, 8) = P_prior(6,8)*c7+P_prior(7,8)*c12+P_prior(8,8)*c13;
		P_post(7, 0) = P_post(0, 7);
		P_post(7, 1) = P_post(1, 7);
		P_post(7, 2) = P_post(2, 7);
		P_post(7, 3) = P_post(3, 7);
		P_post(7, 4) = P_post(4, 7);
		P_post(7, 5) = P_post(5, 7);
		P_post(7, 6) = P_post(6, 7);
		P_post(7, 7) = P_prior(6,7)*c11+P_prior(7,7)*c6+P_prior(8,7)*c10;
		P_post(7, 8) = P_prior(6,8)*c11+P_prior(7,8)*c6+P_prior(8,8)*c10;
		P_post(8, 0) = P_post(0, 8);
		P_post(8, 1) = P_post(1, 8);
		P_post(8, 2) = P_post(2, 8);
		P_post(8, 3) = P_post(3, 8);
		P_post(8, 4) = P_post(4, 8);
		P_post(8, 5) = P_post(5, 8);
		P_post(8, 6) = P_post(6, 8);
		P_post(8, 7) = P_post(7, 8);
		P_post(8, 8) = P_prior(6,8)*c9+P_prior(7,8)*c8+P_prior(8,8)*c5;
	}

	/* P_post becoms the P_prior of other measurement's correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);

	/* error state injection */
	float q_error[4];
	q_error[0] = 1.0f;
	q_error[1] = 0.0f; //0.5 * mat_data(error_state)[6];
	q_error[2] = 0.0f; //0.5 * mat_data(error_state)[7];
	q_error[3] = 0.5 * mat_data(error_state)[8];

	//nominal_state (a posteriori) = q_error * nominal_state (a priori)
	float q_last[4];
	quaternion_copy(q_last, &mat_data(nominal_state)[6]);
	quaternion_mult(q_last, q_error, &mat_data(nominal_state)[6]);

	//renormailization
	quat_normalize(&mat_data(nominal_state)[6]);

	/*=================================================*
	 * convert estimated quaternion to R and Rt matrix *
	 *=================================================*/
	float *q = &mat_data(nominal_state)[6];
	quat_to_rotation_matrix(q, mat_data(_R), mat_data(_Rt));
}

void ins_eskf_gps_correct(float px_ned, float py_ned,
                          float vx_ned, float vy_ned)
{
	float px_gps = px_ned;
	float py_gps = py_ned;
	float vx_gps = vx_ned;
	float vy_gps = vy_ned;
	float px = mat_data(nominal_state)[0];
	float py = mat_data(nominal_state)[1];
	float vx = mat_data(nominal_state)[3];
	float vy = mat_data(nominal_state)[4];

	/* codeblock for preventing nameing conflict */
	{
		/* calculate P * Ht */
		PHt_gps(0, 0) = P_prior(0,0);
		PHt_gps(0, 1) = P_prior(0,1);
		PHt_gps(0, 2) = P_prior(0,3);
		PHt_gps(0, 3) = P_prior(0,4);
		PHt_gps(1, 0) = P_prior(1,0);
		PHt_gps(1, 1) = P_prior(1,1);
		PHt_gps(1, 2) = P_prior(1,3);
		PHt_gps(1, 3) = P_prior(1,4);
		PHt_gps(2, 0) = P_prior(2,0);
		PHt_gps(2, 1) = P_prior(2,1);
		PHt_gps(2, 2) = P_prior(2,3);
		PHt_gps(2, 3) = P_prior(2,4);
		PHt_gps(3, 0) = P_prior(3,0);
		PHt_gps(3, 1) = P_prior(3,1);
		PHt_gps(3, 2) = P_prior(3,3);
		PHt_gps(3, 3) = P_prior(3,4);
		PHt_gps(4, 0) = P_prior(4,0);
		PHt_gps(4, 1) = P_prior(4,1);
		PHt_gps(4, 2) = P_prior(4,3);
		PHt_gps(4, 3) = P_prior(4,4);
		PHt_gps(5, 0) = P_prior(5,0);
		PHt_gps(5, 1) = P_prior(5,1);
		PHt_gps(5, 2) = P_prior(5,3);
		PHt_gps(5, 3) = P_prior(5,4);
		PHt_gps(6, 0) = P_prior(6,0);
		PHt_gps(6, 1) = P_prior(6,1);
		PHt_gps(6, 2) = P_prior(6,3);
		PHt_gps(6, 3) = P_prior(6,4);
		PHt_gps(7, 0) = P_prior(7,0);
		PHt_gps(7, 1) = P_prior(7,1);
		PHt_gps(7, 2) = P_prior(7,3);
		PHt_gps(7, 3) = P_prior(7,4);
		PHt_gps(8, 0) = P_prior(8,0);
		PHt_gps(8, 1) = P_prior(8,1);
		PHt_gps(8, 2) = P_prior(8,3);
		PHt_gps(8, 3) = P_prior(8,4);
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate (H * P * Ht) + V */
		HPHt_V_gps(0, 0) = PHt_gps(0,0)+V_gps(0,0);
		HPHt_V_gps(0, 1) = PHt_gps(0,1);
		HPHt_V_gps(0, 2) = PHt_gps(0,2);
		HPHt_V_gps(0, 3) = PHt_gps(0,3);
		HPHt_V_gps(1, 0) = PHt_gps(1,0);
		HPHt_V_gps(1, 1) = PHt_gps(1,1)+V_gps(1,1);
		HPHt_V_gps(1, 2) = PHt_gps(1,2);
		HPHt_V_gps(1, 3) = PHt_gps(1,3);
		HPHt_V_gps(2, 0) = PHt_gps(3,0);
		HPHt_V_gps(2, 1) = PHt_gps(3,1);
		HPHt_V_gps(2, 2) = PHt_gps(3,2)+V_gps(2,2);
		HPHt_V_gps(2, 3) = PHt_gps(3,3);
		HPHt_V_gps(3, 0) = PHt_gps(4,0);
		HPHt_V_gps(3, 1) = PHt_gps(4,1);
		HPHt_V_gps(3, 2) = PHt_gps(4,2);
		HPHt_V_gps(3, 3) = PHt_gps(4,3)+V_gps(3,3);
	}

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_INV(&_HPHt_V_gps, &_HPHt_V_gps_inv);
	MAT_MULT(&_PHt_gps, &_HPHt_V_gps_inv, &_K_gps);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate error state residual */
		float c0 = vy-vy_gps;
		float c1 = vx-vx_gps;
		float c2 = py-py_gps;
		float c3 = px-px_gps;

		mat_data(error_state)[0] = -K_gps(0,0)*c3-K_gps(0,1)*c2-K_gps(0,2)*c1-K_gps(0,3)*c0;
		mat_data(error_state)[1] = -K_gps(1,0)*c3-K_gps(1,1)*c2-K_gps(1,2)*c1-K_gps(1,3)*c0;
		mat_data(error_state)[3] = -K_gps(3,0)*c3-K_gps(3,1)*c2-K_gps(3,2)*c1-K_gps(3,3)*c0;
		mat_data(error_state)[4] = -K_gps(4,0)*c3-K_gps(4,1)*c2-K_gps(4,2)*c1-K_gps(4,3)*c0;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate a posteriori process covariance matrix */
		float c0 = K_gps(4,3)-1.0;
		float c1 = K_gps(3,2)-1.0;
		float c2 = K_gps(1,1)-1.0;
		float c3 = K_gps(0,0)-1.0;

		P_post(0, 0) = -P_prior(0,0)*c3-K_gps(0,1)*P_prior(1,0)-K_gps(0,2)*P_prior(3,0)-K_gps(0,3)*P_prior(4,0);
		P_post(0, 1) = -P_prior(0,1)*c3-K_gps(0,1)*P_prior(1,1)-K_gps(0,2)*P_prior(3,1)-K_gps(0,3)*P_prior(4,1);
		P_post(0, 2) = -P_prior(0,2)*c3-K_gps(0,1)*P_prior(1,2)-K_gps(0,2)*P_prior(3,2)-K_gps(0,3)*P_prior(4,2);
		P_post(0, 3) = -P_prior(0,3)*c3-K_gps(0,1)*P_prior(1,3)-K_gps(0,2)*P_prior(3,3)-K_gps(0,3)*P_prior(4,3);
		P_post(0, 4) = -P_prior(0,4)*c3-K_gps(0,1)*P_prior(1,4)-K_gps(0,2)*P_prior(3,4)-K_gps(0,3)*P_prior(4,4);
		P_post(0, 5) = -P_prior(0,5)*c3-K_gps(0,1)*P_prior(1,5)-K_gps(0,2)*P_prior(3,5)-K_gps(0,3)*P_prior(4,5);
		P_post(0, 6) = -P_prior(0,6)*c3-K_gps(0,1)*P_prior(1,6)-K_gps(0,2)*P_prior(3,6)-K_gps(0,3)*P_prior(4,6);
		P_post(0, 7) = -P_prior(0,7)*c3-K_gps(0,1)*P_prior(1,7)-K_gps(0,2)*P_prior(3,7)-K_gps(0,3)*P_prior(4,7);
		P_post(0, 8) = -P_prior(0,8)*c3-K_gps(0,1)*P_prior(1,8)-K_gps(0,2)*P_prior(3,8)-K_gps(0,3)*P_prior(4,8);
		P_post(1, 0) = P_post(0, 1);
		P_post(1, 1) = -P_prior(1,1)*c2-K_gps(1,0)*P_prior(0,1)-K_gps(1,2)*P_prior(3,1)-K_gps(1,3)*P_prior(4,1);
		P_post(1, 2) = -P_prior(1,2)*c2-K_gps(1,0)*P_prior(0,2)-K_gps(1,2)*P_prior(3,2)-K_gps(1,3)*P_prior(4,2);
		P_post(1, 3) = -P_prior(1,3)*c2-K_gps(1,0)*P_prior(0,3)-K_gps(1,2)*P_prior(3,3)-K_gps(1,3)*P_prior(4,3);
		P_post(1, 4) = -P_prior(1,4)*c2-K_gps(1,0)*P_prior(0,4)-K_gps(1,2)*P_prior(3,4)-K_gps(1,3)*P_prior(4,4);
		P_post(1, 5) = -P_prior(1,5)*c2-K_gps(1,0)*P_prior(0,5)-K_gps(1,2)*P_prior(3,5)-K_gps(1,3)*P_prior(4,5);
		P_post(1, 6) = -P_prior(1,6)*c2-K_gps(1,0)*P_prior(0,6)-K_gps(1,2)*P_prior(3,6)-K_gps(1,3)*P_prior(4,6);
		P_post(1, 7) = -P_prior(1,7)*c2-K_gps(1,0)*P_prior(0,7)-K_gps(1,2)*P_prior(3,7)-K_gps(1,3)*P_prior(4,7);
		P_post(1, 8) = -P_prior(1,8)*c2-K_gps(1,0)*P_prior(0,8)-K_gps(1,2)*P_prior(3,8)-K_gps(1,3)*P_prior(4,8);
		P_post(2, 0) = P_post(0, 2);
		P_post(2, 1) = P_post(1, 2);
		P_post(2, 2) = P_prior(2,2)-K_gps(2,0)*P_prior(0,2)-K_gps(2,1)*P_prior(1,2)-K_gps(2,2)*P_prior(3,2)-K_gps(2,3)*P_prior(4,2);
		P_post(2, 3) = P_prior(2,3)-K_gps(2,0)*P_prior(0,3)-K_gps(2,1)*P_prior(1,3)-K_gps(2,2)*P_prior(3,3)-K_gps(2,3)*P_prior(4,3);
		P_post(2, 4) = P_prior(2,4)-K_gps(2,0)*P_prior(0,4)-K_gps(2,1)*P_prior(1,4)-K_gps(2,2)*P_prior(3,4)-K_gps(2,3)*P_prior(4,4);
		P_post(2, 5) = P_prior(2,5)-K_gps(2,0)*P_prior(0,5)-K_gps(2,1)*P_prior(1,5)-K_gps(2,2)*P_prior(3,5)-K_gps(2,3)*P_prior(4,5);
		P_post(2, 6) = P_prior(2,6)-K_gps(2,0)*P_prior(0,6)-K_gps(2,1)*P_prior(1,6)-K_gps(2,2)*P_prior(3,6)-K_gps(2,3)*P_prior(4,6);
		P_post(2, 7) = P_prior(2,7)-K_gps(2,0)*P_prior(0,7)-K_gps(2,1)*P_prior(1,7)-K_gps(2,2)*P_prior(3,7)-K_gps(2,3)*P_prior(4,7);
		P_post(2, 8) = P_prior(2,8)-K_gps(2,0)*P_prior(0,8)-K_gps(2,1)*P_prior(1,8)-K_gps(2,2)*P_prior(3,8)-K_gps(2,3)*P_prior(4,8);
		P_post(3, 0) = P_post(0, 3);
		P_post(3, 1) = P_post(1, 3);
		P_post(3, 2) = P_post(2, 3);
		P_post(3, 3) = -P_prior(3,3)*c1-K_gps(3,0)*P_prior(0,3)-K_gps(3,1)*P_prior(1,3)-K_gps(3,3)*P_prior(4,3);
		P_post(3, 4) = -P_prior(3,4)*c1-K_gps(3,0)*P_prior(0,4)-K_gps(3,1)*P_prior(1,4)-K_gps(3,3)*P_prior(4,4);
		P_post(3, 5) = -P_prior(3,5)*c1-K_gps(3,0)*P_prior(0,5)-K_gps(3,1)*P_prior(1,5)-K_gps(3,3)*P_prior(4,5);
		P_post(3, 6) = -P_prior(3,6)*c1-K_gps(3,0)*P_prior(0,6)-K_gps(3,1)*P_prior(1,6)-K_gps(3,3)*P_prior(4,6);
		P_post(3, 7) = -P_prior(3,7)*c1-K_gps(3,0)*P_prior(0,7)-K_gps(3,1)*P_prior(1,7)-K_gps(3,3)*P_prior(4,7);
		P_post(3, 8) = -P_prior(3,8)*c1-K_gps(3,0)*P_prior(0,8)-K_gps(3,1)*P_prior(1,8)-K_gps(3,3)*P_prior(4,8);
		P_post(4, 0) = P_post(0, 4);
		P_post(4, 1) = P_post(1, 4);
		P_post(4, 2) = P_post(2, 4);
		P_post(4, 3) = P_post(3, 4);
		P_post(4, 4) = -P_prior(4,4)*c0-K_gps(4,0)*P_prior(0,4)-K_gps(4,1)*P_prior(1,4)-K_gps(4,2)*P_prior(3,4);
		P_post(4, 5) = -P_prior(4,5)*c0-K_gps(4,0)*P_prior(0,5)-K_gps(4,1)*P_prior(1,5)-K_gps(4,2)*P_prior(3,5);
		P_post(4, 6) = -P_prior(4,6)*c0-K_gps(4,0)*P_prior(0,6)-K_gps(4,1)*P_prior(1,6)-K_gps(4,2)*P_prior(3,6);
		P_post(4, 7) = -P_prior(4,7)*c0-K_gps(4,0)*P_prior(0,7)-K_gps(4,1)*P_prior(1,7)-K_gps(4,2)*P_prior(3,7);
		P_post(4, 8) = -P_prior(4,8)*c0-K_gps(4,0)*P_prior(0,8)-K_gps(4,1)*P_prior(1,8)-K_gps(4,2)*P_prior(3,8);
		P_post(5, 0) = P_post(0, 5);
		P_post(5, 1) = P_post(1, 5);
		P_post(5, 2) = P_post(2, 5);
		P_post(5, 3) = P_post(3, 5);
		P_post(5, 4) = P_post(4, 5);
		P_post(5, 5) = P_prior(5,5)-K_gps(5,0)*P_prior(0,5)-K_gps(5,1)*P_prior(1,5)-K_gps(5,2)*P_prior(3,5)-K_gps(5,3)*P_prior(4,5);
		P_post(5, 6) = P_prior(5,6)-K_gps(5,0)*P_prior(0,6)-K_gps(5,1)*P_prior(1,6)-K_gps(5,2)*P_prior(3,6)-K_gps(5,3)*P_prior(4,6);
		P_post(5, 7) = P_prior(5,7)-K_gps(5,0)*P_prior(0,7)-K_gps(5,1)*P_prior(1,7)-K_gps(5,2)*P_prior(3,7)-K_gps(5,3)*P_prior(4,7);
		P_post(5, 8) = P_prior(5,8)-K_gps(5,0)*P_prior(0,8)-K_gps(5,1)*P_prior(1,8)-K_gps(5,2)*P_prior(3,8)-K_gps(5,3)*P_prior(4,8);
		P_post(6, 0) = P_post(0, 6);
		P_post(6, 1) = P_post(1, 6);
		P_post(6, 2) = P_post(2, 6);
		P_post(6, 3) = P_post(3, 6);
		P_post(6, 4) = P_post(4, 6);
		P_post(6, 5) = P_post(5, 6);
		P_post(6, 6) = P_prior(6,6)-K_gps(6,0)*P_prior(0,6)-K_gps(6,1)*P_prior(1,6)-K_gps(6,2)*P_prior(3,6)-K_gps(6,3)*P_prior(4,6);
		P_post(6, 7) = P_prior(6,7)-K_gps(6,0)*P_prior(0,7)-K_gps(6,1)*P_prior(1,7)-K_gps(6,2)*P_prior(3,7)-K_gps(6,3)*P_prior(4,7);
		P_post(6, 8) = P_prior(6,8)-K_gps(6,0)*P_prior(0,8)-K_gps(6,1)*P_prior(1,8)-K_gps(6,2)*P_prior(3,8)-K_gps(6,3)*P_prior(4,8);
		P_post(7, 0) = P_post(0, 7);
		P_post(7, 1) = P_post(1, 7);
		P_post(7, 2) = P_post(2, 7);
		P_post(7, 3) = P_post(3, 7);
		P_post(7, 4) = P_post(4, 7);
		P_post(7, 5) = P_post(5, 7);
		P_post(7, 6) = P_post(6, 7);
		P_post(7, 7) = P_prior(7,7)-K_gps(7,0)*P_prior(0,7)-K_gps(7,1)*P_prior(1,7)-K_gps(7,2)*P_prior(3,7)-K_gps(7,3)*P_prior(4,7);
		P_post(7, 8) = P_prior(7,8)-K_gps(7,0)*P_prior(0,8)-K_gps(7,1)*P_prior(1,8)-K_gps(7,2)*P_prior(3,8)-K_gps(7,3)*P_prior(4,8);
		P_post(8, 0) = P_post(0, 8);
		P_post(8, 1) = P_post(1, 8);
		P_post(8, 2) = P_post(2, 8);
		P_post(8, 3) = P_post(3, 8);
		P_post(8, 4) = P_post(4, 8);
		P_post(8, 5) = P_post(5, 8);
		P_post(8, 6) = P_post(6, 8);
		P_post(8, 7) = P_post(7, 8);
		P_post(8, 8) = P_prior(8,8)-K_gps(8,0)*P_prior(0,8)-K_gps(8,1)*P_prior(1,8)-K_gps(8,2)*P_prior(3,8)-K_gps(8,3)*P_prior(4,8);
	}

	/* P_post becoms the P_prior of other measurement's correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);

	/* error state injection */
	mat_data(nominal_state)[0] += mat_data(error_state)[0];
	mat_data(nominal_state)[1] += mat_data(error_state)[1];
	mat_data(nominal_state)[3] += mat_data(error_state)[3];
	mat_data(nominal_state)[4] += mat_data(error_state)[4];
}

void ins_eskf_barometer_correct(float barometer_z, float barometer_vz)
{
	float pz_baro = barometer_z;
	float vz_baro = barometer_vz;
	float pz = mat_data(nominal_state)[2];
	float vz = mat_data(nominal_state)[5];

	/* codeblock for preventing nameing conflict */
	{
		/* calculate P * Ht */
		PHt_baro(0, 0) = P_prior(0,2);
		PHt_baro(0, 1) = P_prior(0,5);
		PHt_baro(1, 0) = P_prior(1,2);
		PHt_baro(1, 1) = P_prior(1,5);
		PHt_baro(2, 0) = P_prior(2,2);
		PHt_baro(2, 1) = P_prior(2,5);
		PHt_baro(3, 0) = P_prior(3,2);
		PHt_baro(3, 1) = P_prior(3,5);
		PHt_baro(4, 0) = P_prior(4,2);
		PHt_baro(4, 1) = P_prior(4,5);
		PHt_baro(5, 0) = P_prior(5,2);
		PHt_baro(5, 1) = P_prior(5,5);
		PHt_baro(6, 0) = P_prior(6,2);
		PHt_baro(6, 1) = P_prior(6,5);
		PHt_baro(7, 0) = P_prior(7,2);
		PHt_baro(7, 1) = P_prior(7,5);
		PHt_baro(8, 0) = P_prior(8,2);
		PHt_baro(8, 1) = P_prior(8,5);
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate (H * P * Ht) + V */
		HPHt_V_baro(0, 0) = PHt_baro(2,0)+V_baro(0,0);
		HPHt_V_baro(0, 1) = PHt_baro(2,1);
		HPHt_V_baro(1, 0) = PHt_baro(5,0);
		HPHt_V_baro(1, 1) = PHt_baro(5,1)+V_baro(1,1);
	}

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_INV(&_HPHt_V_baro, &_HPHt_V_baro_inv);
	MAT_MULT(&_PHt_baro, &_HPHt_V_baro_inv, &_K_baro);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate error state residual */
		float c0 = vz-vz_baro;
		float c1 = pz-pz_baro;

		mat_data(error_state)[2] = -K_baro(2,0)*c1-K_baro(2,1)*c0;
		mat_data(error_state)[5] = -K_baro(5,0)*c1-K_baro(5,1)*c0;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate a posteriori process covariance matrix */
		float c0 = K_baro(5,1)-1.0;
		float c1 = K_baro(2,0)-1.0;

		P_post(0, 0) = P_prior(0,0)-K_baro(0,0)*P_prior(2,0)-K_baro(0,1)*P_prior(5,0);
		P_post(0, 1) = P_prior(0,1)-K_baro(0,0)*P_prior(2,1)-K_baro(0,1)*P_prior(5,1);
		P_post(0, 2) = P_prior(0,2)-K_baro(0,0)*P_prior(2,2)-K_baro(0,1)*P_prior(5,2);
		P_post(0, 3) = P_prior(0,3)-K_baro(0,0)*P_prior(2,3)-K_baro(0,1)*P_prior(5,3);
		P_post(0, 4) = P_prior(0,4)-K_baro(0,0)*P_prior(2,4)-K_baro(0,1)*P_prior(5,4);
		P_post(0, 5) = P_prior(0,5)-K_baro(0,0)*P_prior(2,5)-K_baro(0,1)*P_prior(5,5);
		P_post(0, 6) = P_prior(0,6)-K_baro(0,0)*P_prior(2,6)-K_baro(0,1)*P_prior(5,6);
		P_post(0, 7) = P_prior(0,7)-K_baro(0,0)*P_prior(2,7)-K_baro(0,1)*P_prior(5,7);
		P_post(0, 8) = P_prior(0,8)-K_baro(0,0)*P_prior(2,8)-K_baro(0,1)*P_prior(5,8);
		P_post(1, 0) = P_post(0, 1);
		P_post(1, 1) = P_prior(1,1)-K_baro(1,0)*P_prior(2,1)-K_baro(1,1)*P_prior(5,1);
		P_post(1, 2) = P_prior(1,2)-K_baro(1,0)*P_prior(2,2)-K_baro(1,1)*P_prior(5,2);
		P_post(1, 3) = P_prior(1,3)-K_baro(1,0)*P_prior(2,3)-K_baro(1,1)*P_prior(5,3);
		P_post(1, 4) = P_prior(1,4)-K_baro(1,0)*P_prior(2,4)-K_baro(1,1)*P_prior(5,4);
		P_post(1, 5) = P_prior(1,5)-K_baro(1,0)*P_prior(2,5)-K_baro(1,1)*P_prior(5,5);
		P_post(1, 6) = P_prior(1,6)-K_baro(1,0)*P_prior(2,6)-K_baro(1,1)*P_prior(5,6);
		P_post(1, 7) = P_prior(1,7)-K_baro(1,0)*P_prior(2,7)-K_baro(1,1)*P_prior(5,7);
		P_post(1, 8) = P_prior(1,8)-K_baro(1,0)*P_prior(2,8)-K_baro(1,1)*P_prior(5,8);
		P_post(2, 0) = P_post(0, 2);
		P_post(2, 1) = P_post(1, 2);
		P_post(2, 2) = -P_prior(2,2)*c1-K_baro(2,1)*P_prior(5,2);
		P_post(2, 3) = -P_prior(2,3)*c1-K_baro(2,1)*P_prior(5,3);
		P_post(2, 4) = -P_prior(2,4)*c1-K_baro(2,1)*P_prior(5,4);
		P_post(2, 5) = -P_prior(2,5)*c1-K_baro(2,1)*P_prior(5,5);
		P_post(2, 6) = -P_prior(2,6)*c1-K_baro(2,1)*P_prior(5,6);
		P_post(2, 7) = -P_prior(2,7)*c1-K_baro(2,1)*P_prior(5,7);
		P_post(2, 8) = -P_prior(2,8)*c1-K_baro(2,1)*P_prior(5,8);
		P_post(3, 0) = P_post(0, 3);
		P_post(3, 1) = P_post(1, 3);
		P_post(3, 2) = P_post(2, 3);
		P_post(3, 3) = P_prior(3,3)-K_baro(3,0)*P_prior(2,3)-K_baro(3,1)*P_prior(5,3);
		P_post(3, 4) = P_prior(3,4)-K_baro(3,0)*P_prior(2,4)-K_baro(3,1)*P_prior(5,4);
		P_post(3, 5) = P_prior(3,5)-K_baro(3,0)*P_prior(2,5)-K_baro(3,1)*P_prior(5,5);
		P_post(3, 6) = P_prior(3,6)-K_baro(3,0)*P_prior(2,6)-K_baro(3,1)*P_prior(5,6);
		P_post(3, 7) = P_prior(3,7)-K_baro(3,0)*P_prior(2,7)-K_baro(3,1)*P_prior(5,7);
		P_post(3, 8) = P_prior(3,8)-K_baro(3,0)*P_prior(2,8)-K_baro(3,1)*P_prior(5,8);
		P_post(4, 0) = P_post(0, 4);
		P_post(4, 1) = P_post(1, 4);
		P_post(4, 2) = P_post(2, 4);
		P_post(4, 3) = P_post(3, 4);
		P_post(4, 4) = P_prior(4,4)-K_baro(4,0)*P_prior(2,4)-K_baro(4,1)*P_prior(5,4);
		P_post(4, 5) = P_prior(4,5)-K_baro(4,0)*P_prior(2,5)-K_baro(4,1)*P_prior(5,5);
		P_post(4, 6) = P_prior(4,6)-K_baro(4,0)*P_prior(2,6)-K_baro(4,1)*P_prior(5,6);
		P_post(4, 7) = P_prior(4,7)-K_baro(4,0)*P_prior(2,7)-K_baro(4,1)*P_prior(5,7);
		P_post(4, 8) = P_prior(4,8)-K_baro(4,0)*P_prior(2,8)-K_baro(4,1)*P_prior(5,8);
		P_post(5, 0) = P_post(0, 5);
		P_post(5, 1) = P_post(1, 5);
		P_post(5, 2) = P_post(2, 5);
		P_post(5, 3) = P_post(3, 5);
		P_post(5, 4) = P_post(4, 5);
		P_post(5, 5) = -P_prior(5,5)*c0-K_baro(5,0)*P_prior(2,5);
		P_post(5, 6) = -P_prior(5,6)*c0-K_baro(5,0)*P_prior(2,6);
		P_post(5, 7) = -P_prior(5,7)*c0-K_baro(5,0)*P_prior(2,7);
		P_post(5, 8) = -P_prior(5,8)*c0-K_baro(5,0)*P_prior(2,8);
		P_post(6, 0) = P_post(0, 6);
		P_post(6, 1) = P_post(1, 6);
		P_post(6, 2) = P_post(2, 6);
		P_post(6, 3) = P_post(3, 6);
		P_post(6, 4) = P_post(4, 6);
		P_post(6, 5) = P_post(5, 6);
		P_post(6, 6) = P_prior(6,6)-K_baro(6,0)*P_prior(2,6)-K_baro(6,1)*P_prior(5,6);
		P_post(6, 7) = P_prior(6,7)-K_baro(6,0)*P_prior(2,7)-K_baro(6,1)*P_prior(5,7);
		P_post(6, 8) = P_prior(6,8)-K_baro(6,0)*P_prior(2,8)-K_baro(6,1)*P_prior(5,8);
		P_post(7, 0) = P_post(0, 7);
		P_post(7, 1) = P_post(1, 7);
		P_post(7, 2) = P_post(2, 7);
		P_post(7, 3) = P_post(3, 7);
		P_post(7, 4) = P_post(4, 7);
		P_post(7, 5) = P_post(5, 7);
		P_post(7, 6) = P_post(6, 7);
		P_post(7, 7) = P_prior(7,7)-K_baro(7,0)*P_prior(2,7)-K_baro(7,1)*P_prior(5,7);
		P_post(7, 8) = P_prior(7,8)-K_baro(7,0)*P_prior(2,8)-K_baro(7,1)*P_prior(5,8);
		P_post(8, 0) = P_post(0, 8);
		P_post(8, 1) = P_post(1, 8);
		P_post(8, 2) = P_post(2, 8);
		P_post(8, 3) = P_post(3, 8);
		P_post(8, 4) = P_post(4, 8);
		P_post(8, 5) = P_post(5, 8);
		P_post(8, 6) = P_post(6, 8);
		P_post(8, 7) = P_post(7, 8);
		P_post(8, 8) = P_prior(8,8)-K_rangefinder(8,0)*P_prior(2,8)-K_rangefinder(8,1)*P_prior(5,8);
	}

	/* P_post becoms the P_prior of other measurement's correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);

	/* error state injection */
	mat_data(nominal_state)[2] += mat_data(error_state)[2];
	mat_data(nominal_state)[5] += mat_data(error_state)[5];
}


void ins_eskf_rangefinder_correct(float pz_rangefinder, float vz_rangefinder)
{
	float pz = mat_data(nominal_state)[2];
	float vz = mat_data(nominal_state)[5];

	/* codeblock for preventing nameing conflict */
	{
		/* calculate P * Ht */
		PHt_rangefinder(0, 0) = P_prior(0,2);
		PHt_rangefinder(0, 1) = P_prior(0,5);
		PHt_rangefinder(1, 0) = P_prior(1,2);
		PHt_rangefinder(1, 1) = P_prior(1,5);
		PHt_rangefinder(2, 0) = P_prior(2,2);
		PHt_rangefinder(2, 1) = P_prior(2,5);
		PHt_rangefinder(3, 0) = P_prior(3,2);
		PHt_rangefinder(3, 1) = P_prior(3,5);
		PHt_rangefinder(4, 0) = P_prior(4,2);
		PHt_rangefinder(4, 1) = P_prior(4,5);
		PHt_rangefinder(5, 0) = P_prior(5,2);
		PHt_rangefinder(5, 1) = P_prior(5,5);
		PHt_rangefinder(6, 0) = P_prior(6,2);
		PHt_rangefinder(6, 1) = P_prior(6,5);
		PHt_rangefinder(7, 0) = P_prior(7,2);
		PHt_rangefinder(7, 1) = P_prior(7,5);
		PHt_rangefinder(8, 0) = P_prior(8,2);
		PHt_rangefinder(8, 1) = P_prior(8,5);
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate (H * P * Ht) + V */
		HPHt_V_rangefinder(0, 0) = PHt_rangefinder(2,0)+V_rangefinder(0,0);
		HPHt_V_rangefinder(0, 1) = PHt_rangefinder(2,1);
		HPHt_V_rangefinder(1, 0) = PHt_rangefinder(5,0);
		HPHt_V_rangefinder(1, 1) = PHt_rangefinder(5,1)+V_rangefinder(1,1);
	}

	/* calculate kalman gain */
	//K = P * Ht * inv(H*P*Ht + V)
	MAT_INV(&_HPHt_V_rangefinder, &_HPHt_V_rangefinder_inv);
	MAT_MULT(&_PHt_rangefinder, &_HPHt_V_rangefinder_inv, &_K_rangefinder);

	/* codeblock for preventing nameing conflict */
	{
		/* calculate error state residual */
		float c0 = vz-vz_rangefinder;
		float c1 = pz-pz_rangefinder;

		mat_data(error_state)[2] = -K_rangefinder(2,0)*c1-K_rangefinder(2,1)*c0;
		mat_data(error_state)[5] = -K_rangefinder(5,0)*c1-K_rangefinder(5,1)*c0;
	}

	/* codeblock for preventing nameing conflict */
	{
		/* calculate a posteriori process covariance matrix */
		float c0 = K_rangefinder(5,1)-1.0;
		float c1 = K_rangefinder(2,0)-1.0;

		P_post(0, 0) = P_prior(0,0)-K_rangefinder(0,0)*P_prior(2,0)-K_rangefinder(0,1)*P_prior(5,0);
		P_post(0, 1) = P_prior(0,1)-K_rangefinder(0,0)*P_prior(2,1)-K_rangefinder(0,1)*P_prior(5,1);
		P_post(0, 2) = P_prior(0,2)-K_rangefinder(0,0)*P_prior(2,2)-K_rangefinder(0,1)*P_prior(5,2);
		P_post(0, 3) = P_prior(0,3)-K_rangefinder(0,0)*P_prior(2,3)-K_rangefinder(0,1)*P_prior(5,3);
		P_post(0, 4) = P_prior(0,4)-K_rangefinder(0,0)*P_prior(2,4)-K_rangefinder(0,1)*P_prior(5,4);
		P_post(0, 5) = P_prior(0,5)-K_rangefinder(0,0)*P_prior(2,5)-K_rangefinder(0,1)*P_prior(5,5);
		P_post(0, 6) = P_prior(0,6)-K_rangefinder(0,0)*P_prior(2,6)-K_rangefinder(0,1)*P_prior(5,6);
		P_post(0, 7) = P_prior(0,7)-K_rangefinder(0,0)*P_prior(2,7)-K_rangefinder(0,1)*P_prior(5,7);
		P_post(0, 8) = P_prior(0,8)-K_rangefinder(0,0)*P_prior(2,8)-K_rangefinder(0,1)*P_prior(5,8);
		P_post(1, 0) = P_post(0, 1);
		P_post(1, 1) = P_prior(1,1)-K_rangefinder(1,0)*P_prior(2,1)-K_rangefinder(1,1)*P_prior(5,1);
		P_post(1, 2) = P_prior(1,2)-K_rangefinder(1,0)*P_prior(2,2)-K_rangefinder(1,1)*P_prior(5,2);
		P_post(1, 3) = P_prior(1,3)-K_rangefinder(1,0)*P_prior(2,3)-K_rangefinder(1,1)*P_prior(5,3);
		P_post(1, 4) = P_prior(1,4)-K_rangefinder(1,0)*P_prior(2,4)-K_rangefinder(1,1)*P_prior(5,4);
		P_post(1, 5) = P_prior(1,5)-K_rangefinder(1,0)*P_prior(2,5)-K_rangefinder(1,1)*P_prior(5,5);
		P_post(1, 6) = P_prior(1,6)-K_rangefinder(1,0)*P_prior(2,6)-K_rangefinder(1,1)*P_prior(5,6);
		P_post(1, 7) = P_prior(1,7)-K_rangefinder(1,0)*P_prior(2,7)-K_rangefinder(1,1)*P_prior(5,7);
		P_post(1, 8) = P_prior(1,8)-K_rangefinder(1,0)*P_prior(2,8)-K_rangefinder(1,1)*P_prior(5,8);
		P_post(2, 0) = P_post(0, 2);
		P_post(2, 1) = P_post(1, 2);
		P_post(2, 2) = -P_prior(2,2)*c1-K_rangefinder(2,1)*P_prior(5,2);
		P_post(2, 3) = -P_prior(2,3)*c1-K_rangefinder(2,1)*P_prior(5,3);
		P_post(2, 4) = -P_prior(2,4)*c1-K_rangefinder(2,1)*P_prior(5,4);
		P_post(2, 5) = -P_prior(2,5)*c1-K_rangefinder(2,1)*P_prior(5,5);
		P_post(2, 6) = -P_prior(2,6)*c1-K_rangefinder(2,1)*P_prior(5,6);
		P_post(2, 7) = -P_prior(2,7)*c1-K_rangefinder(2,1)*P_prior(5,7);
		P_post(2, 8) = -P_prior(2,8)*c1-K_rangefinder(2,1)*P_prior(5,8);
		P_post(3, 0) = P_post(0, 3);
		P_post(3, 1) = P_post(1, 3);
		P_post(3, 2) = P_post(2, 3);
		P_post(3, 3) = P_prior(3,3)-K_rangefinder(3,0)*P_prior(2,3)-K_rangefinder(3,1)*P_prior(5,3);
		P_post(3, 4) = P_prior(3,4)-K_rangefinder(3,0)*P_prior(2,4)-K_rangefinder(3,1)*P_prior(5,4);
		P_post(3, 5) = P_prior(3,5)-K_rangefinder(3,0)*P_prior(2,5)-K_rangefinder(3,1)*P_prior(5,5);
		P_post(3, 6) = P_prior(3,6)-K_rangefinder(3,0)*P_prior(2,6)-K_rangefinder(3,1)*P_prior(5,6);
		P_post(3, 7) = P_prior(3,7)-K_rangefinder(3,0)*P_prior(2,7)-K_rangefinder(3,1)*P_prior(5,7);
		P_post(3, 8) = P_prior(3,8)-K_rangefinder(3,0)*P_prior(2,8)-K_rangefinder(3,1)*P_prior(5,8);
		P_post(4, 0) = P_post(0, 4);
		P_post(4, 1) = P_post(1, 4);
		P_post(4, 2) = P_post(2, 4);
		P_post(4, 3) = P_post(3, 4);
		P_post(4, 4) = P_prior(4,4)-K_rangefinder(4,0)*P_prior(2,4)-K_rangefinder(4,1)*P_prior(5,4);
		P_post(4, 5) = P_prior(4,5)-K_rangefinder(4,0)*P_prior(2,5)-K_rangefinder(4,1)*P_prior(5,5);
		P_post(4, 6) = P_prior(4,6)-K_rangefinder(4,0)*P_prior(2,6)-K_rangefinder(4,1)*P_prior(5,6);
		P_post(4, 7) = P_prior(4,7)-K_rangefinder(4,0)*P_prior(2,7)-K_rangefinder(4,1)*P_prior(5,7);
		P_post(4, 8) = P_prior(4,8)-K_rangefinder(4,0)*P_prior(2,8)-K_rangefinder(4,1)*P_prior(5,8);
		P_post(5, 0) = P_post(0, 5);
		P_post(5, 1) = P_post(1, 5);
		P_post(5, 2) = P_post(2, 5);
		P_post(5, 3) = P_post(3, 5);
		P_post(5, 4) = P_post(4, 5);
		P_post(5, 5) = -P_prior(5,5)*c0-K_rangefinder(5,0)*P_prior(2,5);
		P_post(5, 6) = -P_prior(5,6)*c0-K_rangefinder(5,0)*P_prior(2,6);
		P_post(5, 7) = -P_prior(5,7)*c0-K_rangefinder(5,0)*P_prior(2,7);
		P_post(5, 8) = -P_prior(5,8)*c0-K_rangefinder(5,0)*P_prior(2,8);
		P_post(6, 0) = P_post(0, 6);
		P_post(6, 1) = P_post(1, 6);
		P_post(6, 2) = P_post(2, 6);
		P_post(6, 3) = P_post(3, 6);
		P_post(6, 4) = P_post(4, 6);
		P_post(6, 5) = P_post(5, 6);
		P_post(6, 6) = P_prior(6,6)-K_rangefinder(6,0)*P_prior(2,6)-K_rangefinder(6,1)*P_prior(5,6);
		P_post(6, 7) = P_prior(6,7)-K_rangefinder(6,0)*P_prior(2,7)-K_rangefinder(6,1)*P_prior(5,7);
		P_post(6, 8) = P_prior(6,8)-K_rangefinder(6,0)*P_prior(2,8)-K_rangefinder(6,1)*P_prior(5,8);
		P_post(7, 0) = P_post(0, 7);
		P_post(7, 1) = P_post(1, 7);
		P_post(7, 2) = P_post(2, 7);
		P_post(7, 3) = P_post(3, 7);
		P_post(7, 4) = P_post(4, 7);
		P_post(7, 5) = P_post(5, 7);
		P_post(7, 6) = P_post(6, 7);
		P_post(7, 7) = P_prior(7,7)-K_rangefinder(7,0)*P_prior(2,7)-K_rangefinder(7,1)*P_prior(5,7);
		P_post(7, 8) = P_prior(7,8)-K_rangefinder(7,0)*P_prior(2,8)-K_rangefinder(7,1)*P_prior(5,8);
		P_post(8, 0) = P_post(0, 8);
		P_post(8, 1) = P_post(1, 8);
		P_post(8, 2) = P_post(2, 8);
		P_post(8, 3) = P_post(3, 8);
		P_post(8, 4) = P_post(4, 8);
		P_post(8, 5) = P_post(5, 8);
		P_post(8, 6) = P_post(6, 8);
		P_post(8, 7) = P_post(7, 8);
		P_post(8, 8) = P_prior(8,8)-K_baro(8,0)*P_prior(2,8)-K_baro(8,1)*P_prior(5,8);
	}

	/* P_post becoms the P_prior of other measurement's correction */
	memcpy(mat_data(_P_prior), mat_data(_P_post), sizeof(float) * 9 * 9);

	/* error state injection */
	mat_data(nominal_state)[2] += mat_data(error_state)[2];
	mat_data(nominal_state)[5] += mat_data(error_state)[5];
}

void ins_eskf_get_attitude_quaternion(float *q)
{
	q[0] = mat_data(nominal_state)[6];
	q[1] = mat_data(nominal_state)[7];
	q[2] = mat_data(nominal_state)[8];
	q[3] = mat_data(nominal_state)[9];
}

void ins_eskf_get_position_ned(float *pos)
{
	pos[0] = mat_data(nominal_state)[0];
	pos[1] = mat_data(nominal_state)[1];
	pos[2] = mat_data(nominal_state)[2];
}

void ins_eskf_get_velocity_ned(float *vel)
{
	vel[0] = mat_data(nominal_state)[3];
	vel[1] = mat_data(nominal_state)[4];
	vel[2] = mat_data(nominal_state)[5];
}

void ins_eskf_estimate(attitude_t *attitude,
                       float *pos_ned_raw, float *vel_ned_raw,
                       float *pos_ned_fused, float *vel_ned_fused)
{
	float curr_time, elapsed_time;

	/* we can't do full state estimation if sensors are not all ready */
	if(ins_eskf_sensor_all_ready() == false) {
		return;
	}

	/* prepare imu data */
	float accel[3];
	float gyro[3], gyro_rad[3];
	get_accel_lpf(accel);
	get_gyro_lpf(gyro);
	gyro_rad[0] = deg_to_rad(gyro[0]);
	gyro_rad[1] = deg_to_rad(gyro[1]);
	gyro_rad[2] = deg_to_rad(gyro[2]);

	/* eskf prediction (400Hz) */
	ins_eskf_predict(accel, gyro_rad);

	/* accelerometer (gravity) correction for attitude states (400Hz) */
	ins_eskf_accelerometer_correct(accel);

	/* compass correction (40Hz)*/
	static bool compass_init = false;
	bool recvd_compass = ins_compass_sync_buffer_available();
	if(recvd_compass == true) {
		/* get megnetometer data from sync buffer */
		float mag[3];
		ins_compass_sync_buffer_pop(mag);

		if(compass_init == false) {
			compass_init = true;
		} else {
			if(ahrs_compass_quality_test(mag) == true) {
				ins_eskf_magnetometer_correct(mag);
			}
		}

		/* calculate correct frequency */
		curr_time = get_sys_time_s();
		elapsed_time = curr_time - ins_eskf.mag_time_last;
		ins_eskf.mag_correct_freq = 1.0f / elapsed_time;
		ins_eskf.mag_time_last = curr_time;
	}

#if (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
	/* barometer correction (40Hz) */
	bool recvd_barometer = ins_barometer_sync_buffer_available();
	if(recvd_barometer == true) {
		/* get barometer data from sync buffer */
		float barometer_height, barometer_height_rate;
		ins_barometer_sync_buffer_pop(&barometer_height,
		                              &barometer_height_rate);
		pos_ned_raw[2] = -barometer_height;
		vel_ned_raw[2] = -barometer_height_rate;

		ins_eskf_barometer_correct(pos_ned_raw[2], vel_ned_raw[2]);

		/* calculate correct frequency */
		curr_time = get_sys_time_s();
		elapsed_time = curr_time - ins_eskf.baro_time_last;
		ins_eskf.baro_correct_freq = 1.0f / elapsed_time;
		ins_eskf.baro_time_last = curr_time;
	}
#elif (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
	/* rangefinder correction (50Hz) */
	bool recvd_rangefinder = ins_rangefinder_sync_buffer_available();
	if(recvd_rangefinder == true) {
		/* get barometer data from sync buffer */
		float rangefinder_height, rangefinder_height_rate;
		ins_rangefinder_sync_buffer_pop(&rangefinder_height,
		                                &rangefinder_height_rate);
		pos_ned_raw[2] = -rangefinder_height;
		vel_ned_raw[2] = -rangefinder_height_rate;

		ins_eskf_rangefinder_correct(pos_ned_raw[2], vel_ned_raw[2]);

		/* calculate correct frequency */
		curr_time = get_sys_time_s();
		elapsed_time = curr_time - ins_eskf.rangefinder_time_last;
		ins_eskf.rangefinder_correct_freq = 1.0f / elapsed_time;
		ins_eskf.rangefinder_time_last = curr_time;
	}
#endif

	/* gps correction (10Hz) */
	bool recvd_gps = ins_gps_sync_buffer_available();
	if(recvd_gps == true) {
		/* get gps data from sync buffer */
		int32_t longitude, latitude;
		float gps_msl_height;
		float gps_ned_vx, gps_ned_vy, gps_ned_vz;
		ins_gps_sync_buffer_pop(&longitude, &latitude, &gps_msl_height,
		                        &gps_ned_vx, &gps_ned_vy, &gps_ned_vz);

		/* convert gps data from geographic coordinate system to ned frame */
		longitude_latitude_to_ned(pos_ned_raw, longitude, latitude, 0);

		if(gps_home_is_set() == false) {
			set_home_longitude_latitude(longitude, latitude, 0/*barometer_height*/); //XXX
		}

		vel_ned_raw[0] = gps_ned_vx;
		vel_ned_raw[1] = gps_ned_vy;

		ins_eskf_gps_correct(pos_ned_raw[0], pos_ned_raw[1],
		                     vel_ned_raw[0], vel_ned_raw[1]);

		/* calculate correct frequency */
		curr_time = get_sys_time_s();
		elapsed_time = curr_time - ins_eskf.gps_time_last;
		ins_eskf.gps_correct_freq = 1.0f / elapsed_time;
		ins_eskf.gps_time_last = curr_time;
	}

	pos_ned_fused[0] = mat_data(nominal_state)[0];  //px
	pos_ned_fused[1] = mat_data(nominal_state)[1];  //py
	pos_ned_fused[2] = mat_data(nominal_state)[2];  //pz
	vel_ned_fused[0] = mat_data(nominal_state)[3];  //vx
	vel_ned_fused[1] = mat_data(nominal_state)[4];  //vy
	vel_ned_fused[2] = mat_data(nominal_state)[5];  //vz
	attitude->q[0] = mat_data(nominal_state)[6];    //q0
	attitude->q[1] = mat_data(nominal_state)[7];    //q1
	attitude->q[2] = mat_data(nominal_state)[8];    //q2
	attitude->q[3] = mat_data(nominal_state)[9];    //q3

	euler_t euler;
	quat_to_euler(attitude->q, &euler);
	attitude->roll = rad_to_deg(euler.roll);
	attitude->pitch = rad_to_deg(euler.pitch);
	attitude->yaw = rad_to_deg(euler.yaw);

	quat_to_rotation_matrix(attitude->q, attitude->R_b2i, attitude->R_i2b);
}

void send_ins_eskf1_covariance_matrix_debug_message(debug_msg_t *payload)
{
	float p00, p11, p22, p33, p44, p55, p66, p77, p88;
	p00 = P_post(0, 0);
	p11 = P_post(1, 1);
	p22 = P_post(2, 2);
	p33 = P_post(3, 3);
	p44 = P_post(4, 4);
	p55 = P_post(5, 5);
	p66 = P_post(6, 6);
	p77 = P_post(7, 7);
	p88 = P_post(8, 8);

	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_ESKF1_COVARIANCE);
	pack_debug_debug_message_float(&p00, payload);
	pack_debug_debug_message_float(&p11, payload);
	pack_debug_debug_message_float(&p22, payload);
	pack_debug_debug_message_float(&p33, payload);
	pack_debug_debug_message_float(&p44, payload);
	pack_debug_debug_message_float(&p55, payload);
	pack_debug_debug_message_float(&p66, payload);
	pack_debug_debug_message_float(&p77, payload);
	pack_debug_debug_message_float(&p88, payload);
}

void send_ins_eskf_correct_freq_debug_message(debug_msg_t *payload)
{
	pack_debug_debug_message_header(payload, MESSAGE_ID_INS_ESKF_CORRECT_FREQ);
	pack_debug_debug_message_float(&ins_eskf.mag_correct_freq, payload);
	pack_debug_debug_message_float(&ins_eskf.baro_correct_freq, payload);
	pack_debug_debug_message_float(&ins_eskf.rangefinder_correct_freq, payload);
	pack_debug_debug_message_float(&ins_eskf.gps_correct_freq, payload);
}
