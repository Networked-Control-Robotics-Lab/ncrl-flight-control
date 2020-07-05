#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_conf.h"
#include "spi.h"
#include "imu.h"

#define mpu6500_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define mpu6500_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_4)

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
	int16_t gyro_bias[3];
	int16_t accel_bias[3];
	int accel_fs;
	int gyro_fs;
	float accel_scale;
	float gyro_scale;
	bool calib_mode;
	volatile bool init_finished;

	/* calibration */
	float gx_min;
	float gx_max;
	float gy_min;
	float gy_max;
	float gz_min;
	float gz_max;
	float accel_rescale_x;
	float accel_rescale_y;
	float accel_rescale_z;
} mpu6500_t;

void mpu6500_init(imu_t *imu);
void mpu6500_int_handler(void);

void mpu6500_config_scale_calib_setting(float gx_extreme_p, float gx_extreme_n,
                                        float gy_extreme_p, float gy_extreme_n,
                                        float gz_extreme_p, float gz_extreme_n);

void mpu6500_get_raw_accel(float *accel);
void mpu6500_get_filtered_accel(float *accel);
void mpu6500_get_raw_gyro(float *gyro);
void mpu6500_get_filtered_gyro(float *gyro);

void debug_print_mpu6500_accel(void);
void debug_print_mpu6500_unscaled_lpf_accel(void);

#endif
