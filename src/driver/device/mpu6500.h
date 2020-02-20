#ifndef __MPU6500_H__
#define __MPU6500_H__

#include "stm32f4xx_conf.h"
#include "spi.h"
#include "vector.h"
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

#define MPU6500A_2g 0.00059815365f
#define MPU6500A_4g 0.00119630731
#define MPU6500A_8g 0.00239261463f
#define MPU6500A_16g 0.00478522926f

#define MPU6500G_250dps 0.007633587786f
#define MPU6500G_500dps 0.015267175572f
#define MPU6500G_1000dps 0.030487804878f
#define MPU6500G_2000dps 0.060975609756f

#define MPU6500T_85degC 0.00294f

void mpu6500_init(imu_t *imu);
void mpu6500_int_handler(void);

void mpu6500_fix_bias(vector3d_16_t *accel_unscaled, vector3d_16_t *gyro_unscaled);
void mpu6500_accel_convert_to_scale(vector3d_16_t *accel_unscaled, vector3d_f_t *accel_scaled);
void mpu6500_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled, vector3d_f_t *gyro_scaled);

void debug_print_mpu6500_accel(void);

#endif
