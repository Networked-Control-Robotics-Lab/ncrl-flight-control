#include "madgwick_ahrs.h"
#include "geometry_ctl.h"

inline void LPF_float(float *raw, float *filtered, float alpha)
{
	*filtered = (1.0f - alpha) * (*filtered) + alpha *((*raw));
}

void madgwick_init(madgwick_t* madgwick, float sample_rate, float beta)
{
	madgwick->beta = beta;
	madgwick->sampleRate = 1 / sample_rate;
	madgwick->q0 = 1;
	madgwick->q1 = 0;
	madgwick->q2 = 0;
	madgwick->q3 = 0;
}

void MadgwickcalculateAngles(madgwick_t* Madgwick)
{
	Madgwick->Roll_rad = (float) atan2(Madgwick->q0 * Madgwick->q1 + Madgwick->q2 * Madgwick->q3, 0.5f - Madgwick->q1 * Madgwick->q1 - Madgwick->q2 * Madgwick->q2);
	Madgwick->Pitch_rad = (float) asin(-2.0f * (Madgwick->q1 * Madgwick->q3 - Madgwick->q0 * Madgwick->q2));

	Madgwick->Roll = Madgwick->Roll_rad*Madgwick_RAD2DEG(1);
	Madgwick->Pitch = Madgwick->Pitch_rad*Madgwick_RAD2DEG(1);
}

//Acceleometer unit : g
//Gyroscope unit : rad/s
void madgwick_imu_ahrs(madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz)
{
	float q0_dot = 0.5f*(-Madgwick->q1*gx - Madgwick->q2*gy - Madgwick->q3*gz);
	float q1_dot = 0.5f*(Madgwick->q0*gx + Madgwick->q2*gz - Madgwick->q3*gy);
	float q2_dot = 0.5f*(Madgwick->q0*gy - Madgwick->q1*gz + Madgwick->q3*gx);
	float q3_dot = 0.5f*(Madgwick->q0*gz + Madgwick->q1*gy - Madgwick->q2*gx);

	float accel_norm = 1 / (sqrt(ax*ax + ay*ay + az*az));
	ax *= accel_norm;
	ay *= accel_norm;
	az *= accel_norm;

	float _2q0 = 2.0f*Madgwick->q0;
	float _2q1 = 2.0f*Madgwick->q1;
	float _2q2 = 2.0f*Madgwick->q2;
	float _2q3 = 2.0f*Madgwick->q3;
	float _4q0 = 4.0f*Madgwick->q0;
	float _4q1 = 4.0f*Madgwick->q1;
	float _4q2 = 4.0f*Madgwick->q2;
	float _4q3 = 4.0f*Madgwick->q3;
	float q0q0 = Madgwick->q0*Madgwick->q0;
	float q1q1 = Madgwick->q1*Madgwick->q1;
	float q2q2 = Madgwick->q2*Madgwick->q2;
	float q3q3 = Madgwick->q3*Madgwick->q3;
	float q1q1_q2q2 = q1q1 + q2q2;

	//Gradient decent algorithm corrective step
	float g0 = _4q0*q1q1_q2q2 + _2q2*ax - _2q1*ay;
	float g1 = _4q1*q1q1_q2q2 - _2q3*ax - _2q0*ay + _4q1*az;
	float g2 = _4q2*q1q1_q2q2 + _2q0*ax - _2q3*ay + _4q2*az;
	float g3 = _4q3*q1q1_q2q2 - _2q1*ax - _2q2*ay;

	/* normalise step magnitude */
	float g_norm = 1 / (sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3));
	g0 *= g_norm;
	g1 *= g_norm;
	g2 *= g_norm;
	g3 *= g_norm;

	q0_dot -= Madgwick->beta*g0;
	q1_dot -= Madgwick->beta*g1;
	q2_dot -= Madgwick->beta*g2;
	q3_dot -= Madgwick->beta*g3;

	Madgwick->q0 += q0_dot*Madgwick->sampleRate;
	Madgwick->q1 += q1_dot*Madgwick->sampleRate;
	Madgwick->q2 += q2_dot*Madgwick->sampleRate;
	Madgwick->q3 += q3_dot*Madgwick->sampleRate;

	float q_norm = 1 / (sqrt(q0q0 + q1q1 + q2q2 + q3q3));
	Madgwick->q0 *= q_norm;
	Madgwick->q1 *= q_norm;
	Madgwick->q2 *= q_norm;
	Madgwick->q3 *= q_norm;

	MadgwickcalculateAngles(Madgwick);
}

void Madgwick_MARG_AHRS(madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	if ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) {
		float q0_dot = 0.5f*(-Madgwick->q1*gx - Madgwick->q2*gy - Madgwick->q3*gz);
		float q1_dot = 0.5f*(Madgwick->q0*gx + Madgwick->q2*gz - Madgwick->q3*gy);
		float q2_dot = 0.5f*(Madgwick->q0*gy - Madgwick->q1*gz + Madgwick->q3*gx);
		float q3_dot = 0.5f*(Madgwick->q0*gz + Madgwick->q1*gy - Madgwick->q2*gx);

		float accel_norm = 1 / (sqrt(ax*ax + ay*ay + az*az));
		ax *= accel_norm;
		ay *= accel_norm;
		az *= accel_norm;

		float mag_norm = 1 / (sqrt(mx*mx + my*my + mz*mz));
		mx *= mag_norm;
		my *= mag_norm;
		mz *= mag_norm;

		float _2q0mx = 2.0f * Madgwick->q0 * mx;
		float _2q0my = 2.0f * Madgwick->q0 * my;
		float _2q0mz = 2.0f * Madgwick->q0 * mz;
		float _2q1mx = 2.0f * Madgwick->q1 * mx;
		float _2q0 = 2.0f*Madgwick->q0;
		float _2q1 = 2.0f*Madgwick->q1;
		float _2q2 = 2.0f*Madgwick->q2;
		float _2q3 = 2.0f*Madgwick->q3;
		float _4q0 = 4.0f*Madgwick->q0;
		float _4q1 = 4.0f*Madgwick->q1;
		float _4q2 = 4.0f*Madgwick->q2;
		float _4q3 = 4.0f*Madgwick->q3;
		float _2q0q2 = 2.0f * Madgwick->q0 * Madgwick->q2;
		float _2q2q3 = 2.0f * Madgwick->q2 * Madgwick->q3;
		float q0q0 = Madgwick->q0 * Madgwick->q0;
		float q0q1 = Madgwick->q0 * Madgwick->q1;
		float q0q2 = Madgwick->q0 * Madgwick->q2;
		float q0q3 = Madgwick->q0 * Madgwick->q3;
		float q1q1 = Madgwick->q1 * Madgwick->q1;
		float q1q2 = Madgwick->q1 * Madgwick->q2;
		float q1q3 = Madgwick->q1 * Madgwick->q3;
		float q2q2 = Madgwick->q2 * Madgwick->q2;
		float q2q3 = Madgwick->q2 * Madgwick->q3;
		float q3q3 = Madgwick->q3 * Madgwick->q3;
		float q1q1_q2q2 = q1q1 + q2q2;

		/* Reference direction of Earth's magnetic field */
		float hx = mx * q0q0 - _2q0my * Madgwick->q3 + _2q0mz * Madgwick->q2 + mx * q1q1 + _2q1 * my * Madgwick->q2 + _2q1 * mz * Madgwick->q3 - mx * q2q2 - mx * q3q3;
		float hy = _2q0mx * Madgwick->q3 + my * q0q0 - _2q0mz * Madgwick->q1 + _2q1mx * Madgwick->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * Madgwick->q3 - my * q3q3;
		float _2bx = (float) sqrt(hx * hx + hy * hy);
		float _2bz = -_2q0mx * Madgwick->q2 + _2q0my * Madgwick->q1 + mz * q0q0 + _2q1mx * Madgwick->q3 - mz * q1q1 + _2q2 * my * Madgwick->q3 - mz * q2q2 + mz * q3q3;
		float _4bx = 2.0f * _2bx;
		float _4bz = 2.0f * _2bz;

		/* Gradient decent algorithm corrective step */
		float g0 = _4q0*q1q1_q2q2 + _2q2*ax - _2q1*ay - _2bz * Madgwick->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Madgwick->q3 + _2bz * Madgwick->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Madgwick->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		float g1 = _4q1*q1q1_q2q2 - _2q3*ax - _2q0*ay + _4q1*az + _2bz * Madgwick->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Madgwick->q2 + _2bz * Madgwick->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Madgwick->q3 - _4bz * Madgwick->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		float g2 = _4q2*q1q1_q2q2 + _2q0*ax - _2q3*ay + _4q2*az + (-_4bx * Madgwick->q2 - _2bz * Madgwick->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Madgwick->q1 + _2bz * Madgwick->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Madgwick->q0 - _4bz * Madgwick->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		float g3 = _4q3*q1q1_q2q2 - _2q1*ax - _2q2*ay + (-_4bx * Madgwick->q3 + _2bz * Madgwick->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Madgwick->q0 + _2bz * Madgwick->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Madgwick->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

		/* normalise step magnitude */
		float g_norm = 1 / (sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3));
		g0 *= g_norm;
		g1 *= g_norm;
		g2 *= g_norm;
		g3 *= g_norm;

		q0_dot -= Madgwick->beta*g0;
		q1_dot -= Madgwick->beta*g1;
		q2_dot -= Madgwick->beta*g2;
		q3_dot -= Madgwick->beta*g3;

		/* Apply feedback step */
		Madgwick->q0 += q0_dot*Madgwick->sampleRate;
		Madgwick->q1 += q1_dot*Madgwick->sampleRate;
		Madgwick->q2 += q2_dot*Madgwick->sampleRate;
		Madgwick->q3 += q3_dot*Madgwick->sampleRate;

		float q_norm = 1 / (sqrt(q0q0 + q1q1 + q2q2 + q3q3));
		Madgwick->q0 *= q_norm;
		Madgwick->q1 *= q_norm;
		Madgwick->q2 *= q_norm;
		Madgwick->q3 *= q_norm;
	} else {
		/* Update IMU algorithm */
		madgwick_imu_ahrs(Madgwick, ax, ay, az, gx, gy, gz);
	}
	/* Calculate new angles */
	MadgwickcalculateAngles(Madgwick);
}

void heading_Madgwick(madgwick_t* MadgwickIMU, MPU9250_t *MPU9250)
{
	float MagX_rotated=0.0f,MagY_rotated=0.0f;//,MagZ_rotated=0.0f;

	float C_roll = arm_cos_f32(-MadgwickIMU->Roll_rad);
	float S_roll = arm_sin_f32(-MadgwickIMU->Roll_rad);

	float C_pitch = arm_cos_f32(-MadgwickIMU->Pitch_rad);
	float S_pitch = arm_sin_f32(-MadgwickIMU->Pitch_rad);

	MagX_rotated = MPU9250->Mx*(C_pitch)+MPU9250->My*(S_roll)*(S_pitch)-MPU9250->Mz*(C_roll)*(S_pitch);
	MagY_rotated = MPU9250->My*(C_roll)+MPU9250->Mz*(S_roll);

	MadgwickIMU->Yaw_rad = atan2f(MagY_rotated,MagX_rotated);
	LPF_float(&MadgwickIMU->Yaw_rad, &MadgwickIMU->Yaw_rad_filtered, 0.018);

	MadgwickIMU->Yaw_filtered = MadgwickIMU->Yaw_rad_filtered*57.32484076433121f;
	MadgwickIMU->Yaw = MadgwickIMU->Yaw_filtered;

	//MadgwickIMU->Yaw_filtered = lowpass_float(&MadgwickIMU->Yaw_filtered, &MadgwickIMU->Yaw, 0.015);
	//printf("%f\n", MadgwickIMU->Yaw_filtered);
}

