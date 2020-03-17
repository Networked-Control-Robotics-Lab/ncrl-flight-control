#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "mpu6500.h"
#include "lpf.h"
#include "imu.h"

#define MPU6500_ACCEL_SCALE MPU6500A_16g
#define MPU6500_GYRO_SCALE MPU6500G_2000dps

#define IMU_CALIB_SAMPLE_CNT 10000

/* acceleromter calibration */
#define GX_MAX (+2020.0f)
#define GX_MIN (-2111.0f)
#define GY_MAX (+2079.0f)
#define GY_MIN (-2043.0f)
#define GZ_MAX (+2558.0f)
#define GZ_MIN (-2048.0f)
#define ACC_RESCALE_X  (4096.0f / (GX_MAX - GX_MIN))
#define ACC_RESCALE_Y  (4096.0f / (GY_MAX - GY_MIN))
#define ACC_RESCALE_Z  (4096.0f / (GZ_MAX - GZ_MIN))

int16_t gyro_bias[3] = {0, 0, 0};
int16_t accel_bias[3] = {0, 0, 0};

volatile bool mpu6500_init_finished = false;
imu_t *mpu6500;

static uint8_t mpu6500_read_byte(uint8_t address)
{
	uint8_t read;

	mpu6500_chip_select();
	spi_read_write(SPI1, address | 0x80);
	read = spi_read_write(SPI1, 0xff);
	mpu6500_chip_deselect();

	return read;
}

static void mpu6500_write_byte(uint8_t address, uint8_t data)
{
	mpu6500_chip_select();
	spi_read_write(SPI1, address);
	spi_read_write(SPI1, data);
	mpu6500_chip_deselect();
}

static uint8_t mpu6500_read_who_am_i()
{
	uint8_t id = mpu6500_read_byte(MPU6500_WHO_AM_I);
	blocked_delay_ms(5);
	return id;
}

static void mpu6500_reset()
{
	mpu6500_write_byte(MPU6500_PWR_MGMT_1, 0x80);
	blocked_delay_ms(200);
}

static void mpu6500_bias_calc(int16_t *gyro, int16_t *accel)
{
	static int imu_bias_sampling_cnt = IMU_CALIB_SAMPLE_CNT;

	static float _gyro_bias[3] = {0.0f, 0.0f, 0.0f};
	static float _accel_bias[3] = {0.0f, 0.0f, 0.0f};

	_gyro_bias[0] += (float)gyro[0] / (float)IMU_CALIB_SAMPLE_CNT;
	_gyro_bias[1] += (float)gyro[1] / (float)IMU_CALIB_SAMPLE_CNT;
	_gyro_bias[2] += (float)gyro[2] / (float)IMU_CALIB_SAMPLE_CNT;
	_accel_bias[0] += (float)accel[0] / (float)IMU_CALIB_SAMPLE_CNT;
	_accel_bias[1] += (float)accel[1] / (float)IMU_CALIB_SAMPLE_CNT;
	_accel_bias[2] += (float)accel[2] / (float)IMU_CALIB_SAMPLE_CNT;

	imu_bias_sampling_cnt--;
	if(imu_bias_sampling_cnt == 0) {
		gyro_bias[0] = (int16_t)_gyro_bias[0];
		gyro_bias[1] = (int16_t)_gyro_bias[1];
		gyro_bias[2] = (int16_t)_gyro_bias[2];
		accel_bias[0] = (int16_t)_accel_bias[0];
		accel_bias[1] = (int16_t)_accel_bias[1];
		accel_bias[2] = (int16_t)(_accel_bias[2] - (9.8f / MPU6500_ACCEL_SCALE));
		mpu6500_init_finished = true;
	}
}

void mpu6500_init(imu_t *imu)
{
	mpu6500 = imu;

	while((mpu6500_read_who_am_i() != 0x70));
	blocked_delay_ms(100);

	mpu6500_reset();

	mpu6500_write_byte(MPU6500_GYRO_CONFIG, 0x18); //gyro sensing range: +-2000dps
	blocked_delay_ms(100);
	mpu6500_write_byte(MPU6500_ACCEL_CONFIG2, 0x08); //accel update rate: 4KHz, disable internel lpf
	blocked_delay_ms(100);
	mpu6500_write_byte(MPU6500_ACCEL_CONFIG, 0x18); //accel sensing range: +-16g
	blocked_delay_ms(100);
	mpu6500_write_byte(MPU6500_INT_ENABLE, 0x01); //enable data ready interrupt
	blocked_delay_ms(100);

	while(mpu6500_init_finished == false);
}

static void mpu6500_apply_calibration(int16_t *accel_unscaled, int16_t *gyro_unscaled)
{
	/* accelerometer bias */
	//float accel_bx = +750.0f;
	//float accel_by = -410.0f;

	accel_unscaled[0] = ((float)accel_unscaled[0] - accel_bias[0]);
	accel_unscaled[1] = ((float)accel_unscaled[1] - accel_bias[1]);
	accel_unscaled[2] = ((float)accel_unscaled[2] - accel_bias[2]);

	gyro_unscaled[0] -= gyro_bias[0];
	gyro_unscaled[1] -= gyro_bias[1];
	gyro_unscaled[2] -= gyro_bias[2];
}

static void mpu6500_accel_convert_to_scale(int16_t *accel_unscaled, float *accel_scaled)
{
	accel_scaled[0] = (float)accel_unscaled[0] * MPU6500_ACCEL_SCALE;
	accel_scaled[1] = (float)accel_unscaled[1] * MPU6500_ACCEL_SCALE;
	accel_scaled[2] = (float)accel_unscaled[2] * MPU6500_ACCEL_SCALE;
}

void mpu6500_gyro_convert_to_scale(int16_t *gyro_unscaled, float *gyro_scaled)
{
	gyro_scaled[0] = gyro_unscaled[0] * MPU6500_GYRO_SCALE;
	gyro_scaled[1] = gyro_unscaled[1] * MPU6500_GYRO_SCALE;
	gyro_scaled[2] = gyro_unscaled[2] * MPU6500_GYRO_SCALE;
}

void mpu6500_int_handler(void)
{
	uint8_t buffer[14];

	/* read sensor datas via spi */
	mpu6500_chip_select();
	spi_read_write(SPI1, MPU6500_ACCEL_XOUT_H | 0x80);
	buffer[0] = spi_read_write(SPI1, 0xff);
	buffer[1] = spi_read_write(SPI1, 0xff);
	buffer[2] = spi_read_write(SPI1, 0xff);
	buffer[3] = spi_read_write(SPI1, 0xff);
	buffer[4] = spi_read_write(SPI1, 0xff);
	buffer[5] = spi_read_write(SPI1, 0xff);
	buffer[6] = spi_read_write(SPI1, 0xff);
	buffer[7] = spi_read_write(SPI1, 0xff);
	buffer[8] = spi_read_write(SPI1, 0xff);
	buffer[9] = spi_read_write(SPI1, 0xff);
	buffer[10] = spi_read_write(SPI1, 0xff);
	buffer[11] = spi_read_write(SPI1, 0xff);
	buffer[12] = spi_read_write(SPI1, 0xff);
	buffer[13] = spi_read_write(SPI1, 0xff);

	/* composite sensor data */
	mpu6500->accel_unscaled[0] = -((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
	mpu6500->accel_unscaled[1] = -((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
	mpu6500->accel_unscaled[2] = -((int16_t)buffer[4] << 8) | (int16_t)buffer[5];
	mpu6500->temp_unscaled = ((int16_t)buffer[6] << 8) | (int16_t)buffer[7];
	mpu6500->gyro_unscaled[0] = -((int16_t)buffer[8] << 8) | (int16_t)buffer[9];
	mpu6500->gyro_unscaled[1] = -((int16_t)buffer[10] << 8) | (int16_t)buffer[11];
	mpu6500->gyro_unscaled[2] = +((int16_t)buffer[12] << 8) | (int16_t)buffer[13];
	mpu6500_chip_deselect();

	if(mpu6500_init_finished == false) {
		mpu6500_bias_calc(mpu6500->gyro_unscaled, mpu6500->accel_unscaled);
		return;
	}

	/* cancel sensor bias and fix slope */
	mpu6500_apply_calibration(mpu6500->accel_unscaled, mpu6500->gyro_unscaled);

	/* sensor data unit conversion */
	mpu6500_accel_convert_to_scale(mpu6500->accel_unscaled, mpu6500->accel_raw);
	mpu6500_gyro_convert_to_scale(mpu6500->gyro_unscaled, mpu6500->gyro_raw);

	/* low pass filtering for accelerometer, gyroscope do not require this process */
	lpf(mpu6500->accel_raw[0], &(mpu6500->accel_lpf[0]), 0.03);
	lpf(mpu6500->accel_raw[1], &(mpu6500->accel_lpf[1]), 0.03);
	lpf(mpu6500->accel_raw[2], &(mpu6500->accel_lpf[2]), 0.03);
	mpu6500->gyro_lpf[0] = mpu6500->gyro_raw[0];
	mpu6500->gyro_lpf[1] = mpu6500->gyro_raw[1];
	mpu6500->gyro_lpf[2] = mpu6500->gyro_raw[2];
}

void debug_print_mpu6500_accel(void)
{
	char s[100] = {0};

	sprintf(s, "[accel] x:%4d, y:%4d, z:%4d\n\r", mpu6500->accel_unscaled[0],
	        mpu6500->accel_unscaled[1], mpu6500->accel_unscaled[2]);

	uart3_puts(s, strlen(s));
	blocked_delay_ms(100);
}
