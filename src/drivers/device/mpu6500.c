#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "uart.h"
#include "mpu6500.h"
#include "lpf.h"
#include "imu.h"
#include "sys_param.h"
#include "common_list.h"
#include "led.h"

#define IMU_CALIB_SAMPLE_CNT 10000

imu_t *imu_mpu6500;

mpu6500_t mpu6500 = {
	.gyro_bias = {0, 0, 0},
	.accel_bias = {0, 0, 0},
	.accel_fs = MPU6500_GYRO_FS_16G,
	.gyro_fs = MPU6500_GYRO_FS_2000_DPS,
	.calib_mode = false,
	.init_finished = false,
};

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

	_gyro_bias[0] += (float)gyro[0] / (float)IMU_CALIB_SAMPLE_CNT;
	_gyro_bias[1] += (float)gyro[1] / (float)IMU_CALIB_SAMPLE_CNT;
	_gyro_bias[2] += (float)gyro[2] / (float)IMU_CALIB_SAMPLE_CNT;

	imu_bias_sampling_cnt--;
	if(imu_bias_sampling_cnt == 0) {
		mpu6500.gyro_bias[0] = (int16_t)_gyro_bias[0];
		mpu6500.gyro_bias[1] = (int16_t)_gyro_bias[1];
		mpu6500.gyro_bias[2] = (int16_t)_gyro_bias[2];
		mpu6500.init_finished = true;
	}
}

bool mpu6500_calibration_not_finished(void)
{
	float imu_finished_calib = 0;
	get_sys_param_float(IMU_FINISH_CALIB, &imu_finished_calib);
	if(fabs(imu_finished_calib) < 0.001) {
		return true;
	} else {
		return false;
	}

}

void mpu6500_init(imu_t *imu)
{
	imu_mpu6500 = imu;

	while((mpu6500_read_who_am_i() != 0x70));
	blocked_delay_ms(100);

	mpu6500_reset();

	switch(mpu6500.accel_fs) {
	case MPU6500_GYRO_FS_2G:
		mpu6500.accel_scale = 9.8 / 16384.0f;
		mpu6500_write_byte(MPU6500_ACCEL_CONFIG, 0x00);
		break;
	case MPU6500_GYRO_FS_4G:
		mpu6500.accel_scale = 9.8f / 8192.0f;
		mpu6500_write_byte(MPU6500_ACCEL_CONFIG, 0x08);
		break;
	case MPU6500_GYRO_FS_8G:
		mpu6500.accel_scale = 9.8f / 4096.0f;
		mpu6500_write_byte(MPU6500_ACCEL_CONFIG, 0x10);
		break;
	case MPU6500_GYRO_FS_16G:
		mpu6500.accel_scale = 9.8f / 2048.0f;
		mpu6500_write_byte(MPU6500_ACCEL_CONFIG, 0x18);
		break;
	}
	blocked_delay_ms(100);

	/* TODO: generate log when error occured */
	if(mpu6500_calibration_not_finished() == true) {
		mpu6500.accel_rescale_x = 1.0f;
		mpu6500.accel_rescale_y = 1.0f;
		mpu6500.accel_rescale_z = 1.0f;
		mpu6500.accel_bias[0] = 0.0f;
		mpu6500.accel_bias[1] = 0.0f;
		mpu6500.accel_bias[2] = 0.0f;
	} else {
		get_sys_param_float(CAL_ACC0_XSCALE, &mpu6500.accel_rescale_x);
		get_sys_param_float(CAL_ACC0_YSCALE, &mpu6500.accel_rescale_y);
		get_sys_param_float(CAL_ACC0_ZSCALE, &mpu6500.accel_rescale_z);
		get_sys_param_float(CAL_ACC0_XOFF, &mpu6500.accel_bias[0]);
		get_sys_param_float(CAL_ACC0_YOFF, &mpu6500.accel_bias[1]);
		get_sys_param_float(CAL_ACC0_ZOFF, &mpu6500.accel_bias[2]);
	}

	switch(mpu6500.gyro_fs) {
	case MPU6500_GYRO_FS_250_DPS:
		mpu6500.gyro_scale = 1.0f / 131.0f;
		mpu6500_write_byte(MPU6500_GYRO_CONFIG, 0x00);
		break;
	case MPU6500_GYRO_FS_500_DPS:
		mpu6500.gyro_scale = 1.0f / 65.5f;
		mpu6500_write_byte(MPU6500_GYRO_CONFIG, 0x08);
		break;
	case MPU6500_GYRO_FS_1000_DPS:
		mpu6500.gyro_scale = 1.0f / 32.8f;
		mpu6500_write_byte(MPU6500_GYRO_CONFIG, 0x10);
		break;
	case MPU6500_GYRO_FS_2000_DPS:
		mpu6500.gyro_scale = 1.0f / 16.4f;
		mpu6500_write_byte(MPU6500_GYRO_CONFIG, 0x18);
		break;
	}
	blocked_delay_ms(100);

	mpu6500_write_byte(MPU6500_ACCEL_CONFIG2, 0x08); //accel update rate: 4KHz, disable internel lpf
	blocked_delay_ms(100);
	mpu6500_write_byte(MPU6500_INT_ENABLE, 0x01); //enable data ready interrupt
	blocked_delay_ms(100);

	while(mpu6500.init_finished == false);
}

void mpu6500_accel_scale_config_reset_default(void)
{
	set_sys_param_float(IMU_FINISH_CALIB, 0);
	set_sys_param_float(CAL_ACC0_ID, 0);
	set_sys_param_float(CAL_ACC1_ID, 0);
	mpu6500.accel_rescale_x = 1.0f;
	mpu6500.accel_rescale_y = 1.0f;
	mpu6500.accel_rescale_z = 1.0f;
}

void mpu6500_accel_bias_config_reset_default(void)
{
	mpu6500.accel_bias[0] = 0.0f;
	mpu6500.accel_bias[1] = 0.0f;
	mpu6500.accel_bias[2] = 0.0f;
}

static void mpu6500_accel_apply_calibration(float *accel_scaled)
{
	accel_scaled[0] = ((float)accel_scaled[0] * mpu6500.accel_rescale_x) - mpu6500.accel_bias[0];
	accel_scaled[1] = ((float)accel_scaled[1] * mpu6500.accel_rescale_y) - mpu6500.accel_bias[1];
	accel_scaled[2] = ((float)accel_scaled[2] * mpu6500.accel_rescale_z) - mpu6500.accel_bias[2];
}

static void mpu6500_gyro_apply_calibration(int16_t *gyro_unscaled)
{
	gyro_unscaled[0] -= mpu6500.gyro_bias[0];
	gyro_unscaled[1] -= mpu6500.gyro_bias[1];
	gyro_unscaled[2] -= mpu6500.gyro_bias[2];
}

static void mpu6500_accel_convert_to_scale(int16_t *accel_unscaled, float *accel_scaled)
{
	accel_scaled[0] = (float)accel_unscaled[0] * mpu6500.accel_scale;
	accel_scaled[1] = (float)accel_unscaled[1] * mpu6500.accel_scale;
	accel_scaled[2] = (float)accel_unscaled[2] * mpu6500.accel_scale;
}

void mpu6500_gyro_convert_to_scale(int16_t *gyro_unscaled, float *gyro_scaled)
{
	gyro_scaled[0] = gyro_unscaled[0] * mpu6500.gyro_scale;
	gyro_scaled[1] = gyro_unscaled[1] * mpu6500.gyro_scale;
	gyro_scaled[2] = gyro_unscaled[2] * mpu6500.gyro_scale;
}

void mpu6500_temp_convert_to_scale(int16_t *temp_unscaled, float *temp_scaled)
{
	*temp_scaled = *temp_unscaled * MPU6500T_85degC + 21.0f;
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
	imu_mpu6500->accel_unscaled[0] = -((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
	imu_mpu6500->accel_unscaled[1] = -((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
	imu_mpu6500->accel_unscaled[2] = +((int16_t)buffer[4] << 8) | (int16_t)buffer[5];
	imu_mpu6500->temp_unscaled = ((int16_t)buffer[6] << 8) | (int16_t)buffer[7];
	imu_mpu6500->gyro_unscaled[0] = -((int16_t)buffer[8] << 8) | (int16_t)buffer[9];
	imu_mpu6500->gyro_unscaled[1] = -((int16_t)buffer[10] << 8) | (int16_t)buffer[11];
	imu_mpu6500->gyro_unscaled[2] = +((int16_t)buffer[12] << 8) | (int16_t)buffer[13];
	mpu6500_chip_deselect();

	if(mpu6500.init_finished == false) {
		mpu6500_bias_calc(imu_mpu6500->gyro_unscaled, imu_mpu6500->accel_unscaled);
		return;
	}

	/* cancel gyro bias */
	mpu6500_gyro_apply_calibration(imu_mpu6500->gyro_unscaled);

	/* sensor data unit conversion */
	mpu6500_accel_convert_to_scale(imu_mpu6500->accel_unscaled, imu_mpu6500->accel_raw);
	mpu6500_gyro_convert_to_scale(imu_mpu6500->gyro_unscaled, imu_mpu6500->gyro_raw);
	mpu6500_temp_convert_to_scale(&imu_mpu6500->temp_unscaled, &imu_mpu6500->temp_raw);

	/* cancel accel bias and fix slope */
	mpu6500_accel_apply_calibration(imu_mpu6500->accel_raw);

	/* low pass filtering for accelerometer, gyroscope do not require this process */
	lpf(imu_mpu6500->accel_raw[0], &(imu_mpu6500->accel_lpf[0]), 0.03);
	lpf(imu_mpu6500->accel_raw[1], &(imu_mpu6500->accel_lpf[1]), 0.03);
	lpf(imu_mpu6500->accel_raw[2], &(imu_mpu6500->accel_lpf[2]), 0.03);
	imu_mpu6500->gyro_lpf[0] = imu_mpu6500->gyro_raw[0];
	imu_mpu6500->gyro_lpf[1] = imu_mpu6500->gyro_raw[1];
	imu_mpu6500->gyro_lpf[2] = imu_mpu6500->gyro_raw[2];
}

void mpu6500_config_scale_calib_setting(float x_scale, float y_scale, float z_scale)
{
	mpu6500.accel_rescale_x = x_scale;
	mpu6500.accel_rescale_y = y_scale;
	mpu6500.accel_rescale_z = z_scale;
}

void mpu6500_config_offset_calib_setting(float x_offset, float y_offset, float z_offset)
{
	mpu6500.accel_bias[0] = x_offset;
	mpu6500.accel_bias[1] = y_offset;
	mpu6500.accel_bias[2] = z_offset;
}

void mpu6500_get_raw_accel(float *accel)
{
	accel[0] = imu_mpu6500->accel_raw[0];
	accel[1] = imu_mpu6500->accel_raw[1];
	accel[2] = imu_mpu6500->accel_raw[2];
}

void mpu6500_get_filtered_accel(float *accel)
{
	accel[0] = imu_mpu6500->accel_lpf[0];
	accel[1] = imu_mpu6500->accel_lpf[1];
	accel[2] = imu_mpu6500->accel_lpf[2];
}

void mpu6500_get_raw_gyro(float *gyro)
{
	gyro[0] = imu_mpu6500->gyro_raw[0];
	gyro[1] = imu_mpu6500->gyro_raw[1];
	gyro[2] = imu_mpu6500->gyro_raw[2];
}

void mpu6500_get_filtered_gyro(float *gyro)
{
	gyro[0] = imu_mpu6500->gyro_lpf[0];
	gyro[1] = imu_mpu6500->gyro_lpf[1];
	gyro[2] = imu_mpu6500->gyro_lpf[2];
}

void debug_print_mpu6500_accel(void)
{
	char s[100] = {0};

	sprintf(s, "[accel] x:%4d, y:%4d, z:%4d\n\r", imu_mpu6500->accel_unscaled[0],
	        imu_mpu6500->accel_unscaled[1], imu_mpu6500->accel_unscaled[2]);

	uart3_puts(s, strlen(s));
	blocked_delay_ms(100);
}

void debug_print_mpu6500_unscaled_lpf_accel(void)
{
	char s[100] = {0};

	sprintf(s, "[accel] x:%4.0f, y:%4.0f, z:%4.0f\n\r", imu_mpu6500->accel_unscaled_lpf[0],
	        imu_mpu6500->accel_unscaled_lpf[1], imu_mpu6500->accel_unscaled_lpf[2]);

	uart3_puts(s, strlen(s));
	blocked_delay_ms(100);
}
