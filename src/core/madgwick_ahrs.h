#ifndef Madgwick_estimator_H
#define Madgwick_estimator_H

#include <stdint.h>
#include "math.h"

#define Madgwick_RAD2DEG(x) ((x) * 57.2957795f)
#define Madgwick_DEG2RAD(x) ((x) * 0.0174532925f)

typedef struct {
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
	float Mx, My, Mz;
	int16_t Ax_Raw, Ay_Raw, Az_Raw;
	int16_t Gx_Raw, Gy_Raw, Gz_Raw;
	int16_t Mx_Raw, My_Raw, Mz_Raw;

	float AMult, GMult, MMult;
	float magCalibrationX, magCalibrationY, magCalibrationZ;
	float mx, my, mz;
	float magbias[3];
	float accele_local[3];
	float gyro_local[3];
	float filtered_mx, filtered_my, filtered_mz;
	float magscale[3];
	float mag_avg_rad;

	uint8_t I2C_Addr;
	uint8_t I2C_Addr_Mag;
} MPU9250_t;

typedef struct _Madgwick_t {
	float Roll, Roll_rad;
	float Pitch, Pitch_rad;
	float Yaw, Yaw_rad, Yaw_rad_filtered;
	float Yaw_filtered, Yaw_original;
	float beta;
	float q0, q1, q2, q3;
	float sampleRate;
} Madgwick_t;

// Setting some constant
void Madgwick_Init(Madgwick_t* Madgwick, float sampleRate, float beta);

//Use quaternion to calculate roll, pitch, yaw
void MadgwickcalculateAngles(Madgwick_t* Madgwick);

//Acceleometer unit : g
//Gyroscope unit : rad/s
void Madgwick_IMU_AHRS(Madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz);
void Madgwick_MARG_AHRS(Madgwick_t* Madgwick, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void heading_Madgwick(Madgwick_t* MadgwickIMU, MPU9250_t *MPU9250);

#endif

