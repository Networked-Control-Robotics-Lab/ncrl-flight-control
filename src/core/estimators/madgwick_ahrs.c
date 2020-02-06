#include "madgwick_ahrs.h"
#include "arm_math.h"

void madgwick_init(madgwick_t* madgwick, float sample_rate, float beta)
{
	madgwick->beta = beta;
	madgwick->sampleRate = 1.0f / sample_rate;
	madgwick->q0 = 1.0f;
	madgwick->q1 = 0.0f;
	madgwick->q2 = 0.0f;
	madgwick->q3 = 0.0f;
}

void madgwick_imu_ahrs(madgwick_t* madgwick, float ax, float ay, float az, float gx, float gy, float gz)
{
	float q0_dot = 0.5f * (-madgwick->q1*gx - madgwick->q2*gy - madgwick->q3*gz);
	float q1_dot = 0.5f * (madgwick->q0*gx + madgwick->q2*gz - madgwick->q3*gy);
	float q2_dot = 0.5f * (madgwick->q0*gy - madgwick->q1*gz + madgwick->q3*gx);
	float q3_dot = 0.5f * (madgwick->q0*gz + madgwick->q1*gy - madgwick->q2*gx);

	float accel_norm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
	ax *= accel_norm;
	ay *= accel_norm;
	az *= accel_norm;

	float _2q0 = 2.0f * madgwick->q0;
	float _2q1 = 2.0f * madgwick->q1;
	float _2q2 = 2.0f * madgwick->q2;
	float _2q3 = 2.0f * madgwick->q3;
	float _4q0 = 4.0f * madgwick->q0;
	float _4q1 = 4.0f * madgwick->q1;
	float _4q2 = 4.0f * madgwick->q2;
	float _4q3 = 4.0f * madgwick->q3;
	float q0q0 = madgwick->q0*madgwick->q0;
	float q1q1 = madgwick->q1*madgwick->q1;
	float q2q2 = madgwick->q2*madgwick->q2;
	float q3q3 = madgwick->q3*madgwick->q3;
	float q1q1_q2q2 = q1q1 + q2q2;

	/* gradient decent algorithm corrective step */
	float g0 = _4q0*q1q1_q2q2 + _2q2*ax - _2q1*ay;
	float g1 = _4q1*q1q1_q2q2 - _2q3*ax - _2q0*ay + _4q1*az;
	float g2 = _4q2*q1q1_q2q2 + _2q0*ax - _2q3*ay + _4q2*az;
	float g3 = _4q3*q1q1_q2q2 - _2q1*ax - _2q2*ay;

	/* normalize step magnitude */
	float g_norm = 1.0f / sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3);
	g0 *= g_norm;
	g1 *= g_norm;
	g2 *= g_norm;
	g3 *= g_norm;

	q0_dot -= madgwick->beta*g0;
	q1_dot -= madgwick->beta*g1;
	q2_dot -= madgwick->beta*g2;
	q3_dot -= madgwick->beta*g3;

	madgwick->q0 += q0_dot*madgwick->sampleRate;
	madgwick->q1 += q1_dot*madgwick->sampleRate;
	madgwick->q2 += q2_dot*madgwick->sampleRate;
	madgwick->q3 += q3_dot*madgwick->sampleRate;

	float q_norm = 1.0f / sqrt(q0q0 + q1q1 + q2q2 + q3q3);
	madgwick->q0 *= q_norm;
	madgwick->q1 *= q_norm;
	madgwick->q2 *= q_norm;
	madgwick->q3 *= q_norm;
}

void madgwick_MARG_AHRS(madgwick_t* madgwick, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	if((mx == 0.0f) && (my == 0.0f) && mz == 0.0f) {
		madgwick_imu_ahrs(madgwick, ax, ay, az, gx, gy, gz);
		return;
	}

	float q0_dot = 0.5f * (-madgwick->q1*gx - madgwick->q2*gy - madgwick->q3*gz);
	float q1_dot = 0.5f * (madgwick->q0*gx + madgwick->q2*gz - madgwick->q3*gy);
	float q2_dot = 0.5f * (madgwick->q0*gy - madgwick->q1*gz + madgwick->q3*gx);
	float q3_dot = 0.5f * (madgwick->q0*gz + madgwick->q1*gy - madgwick->q2*gx);

	float accel_norm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
	ax *= accel_norm;
	ay *= accel_norm;
	az *= accel_norm;

	float mag_norm = 1.0f / sqrt(mx*mx + my*my + mz*mz);
	mx *= mag_norm;
	my *= mag_norm;
	mz *= mag_norm;

	float _2q0mx = 2.0f * madgwick->q0 * mx;
	float _2q0my = 2.0f * madgwick->q0 * my;
	float _2q0mz = 2.0f * madgwick->q0 * mz;
	float _2q1mx = 2.0f * madgwick->q1 * mx;
	float _2q0 = 2.0f * madgwick->q0;
	float _2q1 = 2.0f * madgwick->q1;
	float _2q2 = 2.0f * madgwick->q2;
	float _2q3 = 2.0f * madgwick->q3;
	float _4q0 = 4.0f * madgwick->q0;
	float _4q1 = 4.0f * madgwick->q1;
	float _4q2 = 4.0f * madgwick->q2;
	float _4q3 = 4.0f * madgwick->q3;
	float q0q0 = madgwick->q0 * madgwick->q0;
	float q0q1 = madgwick->q0 * madgwick->q1;
	float q0q2 = madgwick->q0 * madgwick->q2;
	float q0q3 = madgwick->q0 * madgwick->q3;
	float q1q1 = madgwick->q1 * madgwick->q1;
	float q1q2 = madgwick->q1 * madgwick->q2;
	float q1q3 = madgwick->q1 * madgwick->q3;
	float q2q2 = madgwick->q2 * madgwick->q2;
	float q2q3 = madgwick->q2 * madgwick->q3;
	float q3q3 = madgwick->q3 * madgwick->q3;
	float q1q1_q2q2 = q1q1 + q2q2;

	/* reference direction of earth's magnetic field */
	float hx = mx * q0q0 - _2q0my * madgwick->q3 + _2q0mz * madgwick->q2 + mx * q1q1 + _2q1 * my * madgwick->q2 + _2q1 * mz * madgwick->q3 - mx * q2q2 - mx * q3q3;
	float hy = _2q0mx * madgwick->q3 + my * q0q0 - _2q0mz * madgwick->q1 + _2q1mx * madgwick->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * madgwick->q3 - my * q3q3;
	float _2bx = (float) sqrt(hx * hx + hy * hy);
	float _2bz = -_2q0mx * madgwick->q2 + _2q0my * madgwick->q1 + mz * q0q0 + _2q1mx * madgwick->q3 - mz * q1q1 + _2q2 * my * madgwick->q3 - mz * q2q2 + mz * q3q3;
	float _4bx = 2.0f * _2bx;
	float _4bz = 2.0f * _2bz;

	/* gradient decent algorithm corrective step */
	float g0 = _4q0*q1q1_q2q2 + _2q2*ax - _2q1*ay - _2bz * madgwick->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * madgwick->q3 + _2bz * madgwick->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * madgwick->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	float g1 = _4q1*q1q1_q2q2 - _2q3*ax - _2q0*ay + _4q1*az + _2bz * madgwick->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * madgwick->q2 + _2bz * madgwick->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * madgwick->q3 - _4bz * madgwick->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	float g2 = _4q2*q1q1_q2q2 + _2q0*ax - _2q3*ay + _4q2*az + (-_4bx * madgwick->q2 - _2bz * madgwick->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * madgwick->q1 + _2bz * madgwick->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * madgwick->q0 - _4bz * madgwick->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	float g3 = _4q3*q1q1_q2q2 - _2q1*ax - _2q2*ay + (-_4bx * madgwick->q3 + _2bz * madgwick->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * madgwick->q0 + _2bz * madgwick->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * madgwick->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

	/* normalize step magnitude */
	float g_norm = 1 / sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3);
	g0 *= g_norm;
	g1 *= g_norm;
	g2 *= g_norm;
	g3 *= g_norm;

	q0_dot -= madgwick->beta * g0;
	q1_dot -= madgwick->beta * g1;
	q2_dot -= madgwick->beta * g2;
	q3_dot -= madgwick->beta * g3;

	/* apply feedback step */
	madgwick->q0 += q0_dot * madgwick->sampleRate;
	madgwick->q1 += q1_dot * madgwick->sampleRate;
	madgwick->q2 += q2_dot * madgwick->sampleRate;
	madgwick->q3 += q3_dot * madgwick->sampleRate;

	float q_norm = 1 / sqrt(q0q0 + q1q1 + q2q2 + q3q3);
	madgwick->q0 *= q_norm;
	madgwick->q1 *= q_norm;
	madgwick->q2 *= q_norm;
	madgwick->q3 *= q_norm;
}
