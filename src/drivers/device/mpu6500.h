#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_conf.h"
#include "spi.h"
#include "imu.h"
#include "proj_config.h"

/* driver mode */
#define USE_MPU6500 1
#define USE_MPU6000 2

#define MPU6500_SMPLRT_DIV 0x19
#define MPU6500_CONFIG 0x1A
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_CONFIG2 0x1D
#define MPU6500_MOT_THR 0x1F
#define MPU6500_FIFO_EN 0x23
#define MPU6500_INT_PIN_CFG 0x37
#define MPU6500_INT_ENABLE 0x38
#define MPU6500_INT_STATUS 0x3A
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_ACCEL_XOUT_L 0x3C
#define MPU6500_ACCEL_YOUT_H 0x3D
#define MPU6500_ACCEL_YOUT_L 0x3E
#define MPU6500_ACCEL_ZOUT_H 0x3F
#define MPU6500_ACCEL_ZOUT_L 0x40
#define MPU6500_TEMP_OUT_H 0x41
#define MPU6500_TEMP_OUT_L 0x42
#define MPU6500_GYRO_XOUT_H 0x43
#define MPU6500_GYRO_XOUT_L 0x44
#define MPU6500_GYRO_YOUT_H 0x45
#define MPU6500_GYRO_YOUT_L 0x46
#define MPU6500_GYRO_ZOUT_H 0x47
#define MPU6500_GYRO_ZOUT_L 0x48
#define MPU6500_USER_CTRL 0x6A
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_PWR_MGMT_2 0x6C
#define MPU6500_FIFO_COUNTH 0x72
#define MPU6500_FIFO_COUNTL 0x73
#define MPU6500_FIFO_R_W 0x74
#define MPU6500_WHO_AM_I 0x75

#define MPU6500_I2C_SLV4_ADDR 0x31
#define MPU6500_I2C_SLV4_REG 0x32
#define MPU6500_I2C_SLV4_CTRL 0x34
#define MPU6500_I2C_SLV4_DI 0x35
#define MPU6500_I2C_SLV4_DO 0x40

#define MPU6500T_85degC 0.00294f

#define GYRO_DLPF_BANDWIDTH_20Hz  0x04

#define ACCEL_DLPF_BANDWIDTH_20Hz 0x04
#define ACCEL_DLPF_DISABLE        0x08

enum {
	MPU6500_GYRO_FS_2G = 0,
	MPU6500_GYRO_FS_4G = 1,
	MPU6500_GYRO_FS_8G = 2,
	MPU6500_GYRO_FS_16G = 3,
} MPU6500_ACCEL_FS;

enum {
	MPU6500_GYRO_FS_250_DPS = 0,
	MPU6500_GYRO_FS_500_DPS = 1,
	MPU6500_GYRO_FS_1000_DPS = 2,
	MPU6500_GYRO_FS_2000_DPS = 3,
} MPU6500_GYRO_FS;

typedef struct {
	/* sensor mode */
	int accel_fs;
	int gyro_fs;

	/* sensor calibration */
	int16_t gyro_bias[3];
	float accel_bias[3];
	float accel_scale;
	float gyro_scale;
	volatile bool init_finished;

	/* sensor datas */
	int16_t accel_unscaled[3];
	int16_t gyro_unscaled[3];
	int16_t temp_unscaled;

	float accel_raw[3];
	float gyro_raw[3];
	float temp_raw;

	float accel_lpf[3];
	float accel_unscaled_lpf[3];
	float gyro_lpf[3];

	/* calibration */
	float accel_rescale_x;
	float accel_rescale_y;
	float accel_rescale_z;
} mpu6500_t;

void mpu6500_init(void);
void mpu6500_int_handler(void);
bool mpu6500_calibration_not_finished(void);

void mpu6500_reset_scale_factor(void);
void mpu6500_reset_bias(void);

void mpu6500_set_scale_factor(float x_scale, float y_scale, float z_scale);
void mpu6500_set_bias(float x_offset, float y_offset, float z_offset);

void mpu6500_get_raw_accel(float *accel);
void mpu6500_get_filtered_accel(float *accel);
void mpu6500_get_gyro_raw(float *gyro);
void mpu6500_get_gyro_lpf(float *gyro);

void debug_print_mpu6500_accel(void);
void debug_print_mpu6500_unscaled_lpf_accel(void);

float mpu6500_get_temperature(void);

#endif
