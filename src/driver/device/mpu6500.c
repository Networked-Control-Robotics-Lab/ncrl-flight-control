#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "mpu6500.h"
#include "vector.h"
#include "lpf.h"
#include "imu.h"

#define MPU6500_ACCEL_SCALE MPU6500A_16g
#define MPU6500_GYRO_SCALE MPU6500G_2000dps

#define GYRO_CALIB_SAMPLE_CNT 10000;

vector3d_16_t gyro_bias  = {0, 0, 0};

int gyro_sample_cnt = GYRO_CALIB_SAMPLE_CNT;

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

static void mpu6500_gyro_bias_calc(vector3d_16_t *gyro)
{
	static float _gyro_bias[3] = {0.0f, 0.0f, 0.0f};

	_gyro_bias[0] += (float)gyro->x / (float)GYRO_CALIB_SAMPLE_CNT;
	_gyro_bias[1] += (float)gyro->y / (float)GYRO_CALIB_SAMPLE_CNT;
	_gyro_bias[2] += (float)gyro->z / (float)GYRO_CALIB_SAMPLE_CNT;

	gyro_sample_cnt--;
	if(gyro_sample_cnt == 0) {
		gyro_bias.x = (int16_t)_gyro_bias[0];
		gyro_bias.y = (int16_t)_gyro_bias[1];
		gyro_bias.z = (int16_t)_gyro_bias[2];
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

static void mpu6500_accel_apply_correction(vector3d_16_t *accel_unscaled, vector3d_16_t *gyro_unscaled)
{
	/* use max/min gravity value to correct the slope rate of accerometer  */
	float gx_max = +2020.0f, gx_min = -2111.0f;
	float gy_max = +2079.0f, gy_min = -2043.0f;
	float gz_max = +2558.0f, gz_min = -2048.0f;

	/* accelerometer bias */
	float accel_bx = +600.0f;  //trim this for pitch angle, + sign for - angle compensation
	float accel_by = -100.0f;  //trim this for roll angle, + sign for + angle compensation
	float accel_bz = 0.0f;

	float rescale_x = 4096.0f / (gx_max - gx_min);
	float rescale_y = 4096.0f / (gy_max - gy_min);
	float rescale_z = 4096.0f / (gz_max - gz_min);

	accel_unscaled->x = ((float)accel_unscaled->x - accel_bx) * rescale_x;
	accel_unscaled->y = ((float)accel_unscaled->y - accel_by) * rescale_y;
	accel_unscaled->z = ((float)accel_unscaled->z - accel_bz) * rescale_z;

	gyro_unscaled->x -= gyro_bias.x;
	gyro_unscaled->y -= gyro_bias.y;
	gyro_unscaled->z -= gyro_bias.z;
}

static void mpu6500_accel_convert_to_scale(vector3d_16_t *accel_unscaled, vector3d_f_t *accel_scaled)
{
	accel_scaled->x = (float)accel_unscaled->x * MPU6500_ACCEL_SCALE;
	accel_scaled->y = (float)accel_unscaled->y * MPU6500_ACCEL_SCALE;
	accel_scaled->z = (float)accel_unscaled->z * MPU6500_ACCEL_SCALE;
}

void mpu6500_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled, vector3d_f_t *gyro_scaled)
{
	gyro_scaled->x = gyro_unscaled->x * MPU6500_GYRO_SCALE;
	gyro_scaled->y = gyro_unscaled->y * MPU6500_GYRO_SCALE;
	gyro_scaled->z = gyro_unscaled->z * MPU6500_GYRO_SCALE;
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
	mpu6500->accel_unscaled.x = -((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
	mpu6500->accel_unscaled.y = -((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
	mpu6500->accel_unscaled.z = -((int16_t)buffer[4] << 8) | (int16_t)buffer[5];
	mpu6500->temp_unscaled = ((int16_t)buffer[6] << 8) | (int16_t)buffer[7];
	mpu6500->gyro_unscaled.x = -((int16_t)buffer[8] << 8) | (int16_t)buffer[9];
	mpu6500->gyro_unscaled.y = -((int16_t)buffer[10] << 8) | (int16_t)buffer[11];
	mpu6500->gyro_unscaled.z = +((int16_t)buffer[12] << 8) | (int16_t)buffer[13];

	mpu6500_chip_deselect();

	if(mpu6500_init_finished == false) {
		mpu6500_gyro_bias_calc(&mpu6500->gyro_unscaled);
		return;
	}

#if (DO_CALIBRATION == 0)
	/* cancel sensor bias and fix slope */
	mpu6500_accel_apply_correction(&mpu6500->accel_unscaled, &mpu6500->gyro_unscaled);
#endif

	/* sensor data unit conversion */
	mpu6500_accel_convert_to_scale(&mpu6500->accel_unscaled, &mpu6500->accel_raw);
	mpu6500_gyro_convert_to_scale(&mpu6500->gyro_unscaled, &mpu6500->gyro_raw);

	/* low pass filtering for accelerometer, gyroscope do not require this process */
	lpf(mpu6500->accel_raw.x, &(mpu6500->accel_lpf.x), 0.03);
	lpf(mpu6500->accel_raw.y, &(mpu6500->accel_lpf.y), 0.03);
	lpf(mpu6500->accel_raw.z, &(mpu6500->accel_lpf.z), 0.03);
	mpu6500->gyro_lpf.x = mpu6500->gyro_raw.x;
	mpu6500->gyro_lpf.y = mpu6500->gyro_raw.y;
	mpu6500->gyro_lpf.z = mpu6500->gyro_raw.z;
}

void debug_print_mpu6500_accel(void)
{
	char s[100] = {0};

	sprintf(s, "[accel] x:%4d, y:%4d, z:%4d\n\r", mpu6500->accel_unscaled.x,
	        mpu6500->accel_unscaled.y, mpu6500->accel_unscaled.z);

	uart3_puts(s, strlen(s));
	blocked_delay_ms(100);
}
