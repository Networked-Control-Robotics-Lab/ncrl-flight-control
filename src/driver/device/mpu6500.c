#include "stm32f4xx_conf.h"
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "mpu6500.h"
#include "vector.h"

#define MPU9250_ACCEL_SCALE MPU9250A_8g
#define MPU9250_GYRO_SCALE MPU9250G_1000dps

vector3d_16_t mpu9250_accel_error_bias = {0.0f, 0.0f, 0.0f};
vector3d_16_t mpu9250_gyro_error_bias  = {0.0f, 0.0f, 0.0f};

uint8_t mpu9250_read_byte(uint8_t address)
{
	uint8_t read;

	mpu9250_chip_select();

	spi_read_write(SPI1, address | 0x80);
	read = spi_read_write(SPI1, 0xff);

	mpu9250_chip_deselect();

	return read;
}

void mpu9250_write_byte(uint8_t address, uint8_t data)
{
	mpu9250_chip_select();

	spi_read_write(SPI1, address);
	spi_read_write(SPI1, data);

	mpu9250_chip_deselect();
}

uint8_t mpu9250_read_who_am_i()
{
	uint8_t id;
	id = mpu9250_read_byte(MPU9250_WHO_AM_I);

	return id;
}

void mpu9250_reset()
{
        mpu9250_write_byte(MPU9250_PWR_MGMT_1, 0x80);
	delay_ms(100);
}

void mpu9250_bias_calculate(void)
{
	vector3d_16_t accel_unscaled, gyro_unscaled;
	vector3d_16_t gyro_unscaled_last;

	vector3d_f_t gyro_bias_float = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

	int16_t gyro_recalib_threshold = 30;

	recalibrate:
	mpu9250_read_unscaled_data(&accel_unscaled, &gyro_unscaled);
	gyro_unscaled_last = gyro_unscaled;

	int count = 10000;

	led_on(LED_R);
	led_on(LED_G);
	led_on(LED_B);

	int i;
	for(i = 1; i < count; i++) {
		mpu9250_read_unscaled_data(&accel_unscaled, &gyro_unscaled);

		gyro_bias_float.x += (float)gyro_unscaled.x / (float)count;
		gyro_bias_float.y += (float)gyro_unscaled.y / (float)count;
		gyro_bias_float.z += (float)gyro_unscaled.z / (float)count;

		int16_t gyro_change_x = abs(gyro_unscaled.x - gyro_unscaled_last.x);
		int16_t gyro_change_y = abs(gyro_unscaled.y - gyro_unscaled_last.y);
		int16_t gyro_change_z = abs(gyro_unscaled.z - gyro_unscaled_last.z);

		//printf("dx:%d, dy:%d, dz:%d\n\r", gyro_change_x, gyro_change_y, gyro_change_z);

		if(gyro_change_x > gyro_recalib_threshold || gyro_change_y > gyro_recalib_threshold ||
		   gyro_change_z > gyro_recalib_threshold) {
			goto recalibrate;
		}

		gyro_unscaled_last = gyro_unscaled;
	}

	led_off(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	mpu9250_gyro_error_bias.x = (int16_t)gyro_bias_float.x;
	mpu9250_gyro_error_bias.y = (int16_t)gyro_bias_float.y;
	mpu9250_gyro_error_bias.z = (int16_t)gyro_bias_float.z;
}

int mpu9250_init()
{
	if(mpu9250_read_who_am_i() != 0x71) return 1;

	mpu9250_reset();
	delay_ms(50);
        mpu9250_write_byte(MPU9250_GYRO_CONFIG, 0x10); //gyro: 1000Hz
	delay_ms(50);
        mpu9250_write_byte(MPU9250_ACCEL_CONFIG, 0x10); //accel range: 8g
	delay_ms(50);

        return 0;
}

void mpu9250_read_unscaled_data(vector3d_16_t *accel_unscaled_data, vector3d_16_t *gyro_unscaled_data)
{
	uint16_t temp;
	vector3d_16_t mag_unscaled_data;

	uint8_t buffer[14];

	mpu9250_chip_select();

	spi_read_write(SPI1, MPU9250_ACCEL_XOUT_H | 0x80);
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

	mpu9250_chip_deselect();

	//accelerometer
	accel_unscaled_data->x = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	accel_unscaled_data->y = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
	accel_unscaled_data->z = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[5];
#if 0
	//temperature
	temp = (buffer[6] << 8) | buffer[7];
#endif
	//gyroscope
	gyro_unscaled_data->x = -(buffer[8] << 8) | buffer[9];
	gyro_unscaled_data->y = -(buffer[10] << 8) | buffer[11];
	gyro_unscaled_data->z = -(buffer[12] << 8) | buffer[13];
#if 0
	//magnetometer
	uint8_t ak8963_status1 = buffer[14];
	uint8_t ak8963_status2 = buffer[21];
	mag_unscaled_data.x = (buffer[16] << 8) | buffer[15];
	mag_unscaled_data.y = (buffer[18] << 8) | buffer[17];
	mag_unscaled_data.z = (buffer[20] << 8) | buffer[19];
#endif
}

void mpu9250_fix_bias(vector3d_16_t *accel_unscaled_data,
	vector3d_16_t *gyro_unscaled_data)
{
	accel_unscaled_data->x -= mpu9250_accel_error_bias.x;
	accel_unscaled_data->y -= mpu9250_accel_error_bias.y;
	accel_unscaled_data->z -= mpu9250_accel_error_bias.z;
	gyro_unscaled_data->x -= mpu9250_gyro_error_bias.x;
	gyro_unscaled_data->y -= mpu9250_gyro_error_bias.y;
	gyro_unscaled_data->z -= mpu9250_gyro_error_bias.z;
}

void mpu9250_accel_convert_to_scale(vector3d_16_t *accel_unscaled_data,
	vector3d_f_t *accel_scaled_data)
{
	float bias_x = 0.0f;
	float bias_y = 0.0f;
	float bias_z = 0.0f;

	float rescale_x = 1;
	float rescale_y = 1;
	float rescale_z = 1;

	accel_scaled_data->x = ((float)accel_unscaled_data->x) * rescale_x * MPU9250_ACCEL_SCALE * 10.0f;
	accel_scaled_data->y = ((float)accel_unscaled_data->y) * rescale_y * MPU9250_ACCEL_SCALE * 10.0f;
	accel_scaled_data->z = ((float)accel_unscaled_data->z) * rescale_z * MPU9250_ACCEL_SCALE * 10.0f;
}

void mpu9250_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled_data,
	vector3d_f_t *gyro_scaled_data)
{
	 gyro_scaled_data->x = gyro_unscaled_data->x * MPU9250_GYRO_SCALE;
	 gyro_scaled_data->y = gyro_unscaled_data->y * MPU9250_GYRO_SCALE;
	 gyro_scaled_data->z = gyro_unscaled_data->z * MPU9250_GYRO_SCALE;
}
