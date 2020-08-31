#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"
#include "ist8310.h"
#include "sys_time.h"

SemaphoreHandle_t ist8310_semphr;

ist8310_t ist8310;

uint8_t ist8310_read_byte(uint8_t addr)
{
	uint8_t data;

	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_wait_ack();
	data = sw_i2c_read_byte();
	sw_i2c_nack();
	sw_i2c_stop();

	return data;
}

void ist8310_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(data);
	sw_i2c_wait_ack();
	sw_i2c_stop();
}

void ist8310_read_bytes(uint8_t addr, uint8_t *data, int size)
{
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_wait_ack();
	for(int i = 0; i < size; i++) {
		data[i] = sw_i2c_read_byte();
		if(i == (size-1)) {
			/* send nack if all bytes are received */
			sw_i2c_nack();
		} else {
			/* send ack for requesting next byte */
			sw_i2c_ack();
		}
	}
	sw_i2c_stop();
}

static uint8_t ist8310_read_who_i_am(void)
{
	uint8_t id = ist8310_read_byte(IST8310_REG_WIA);
	blocked_delay_ms(5);
	return id;
}

void ist8130_init(void)
{
	ist8310_semphr = xSemaphoreCreateBinary();

	while(ist8310_read_who_i_am() != IST8310_CHIP_ID);

	ist8310_write_byte(IST8310_REG_CTRL1, IST8310_ODR_50HZ);
	blocked_delay_ms(10);

	ist8310_write_byte(IST8310_REG_AVG, IST8310_AVG_16);
	blocked_delay_ms(10);

	ist8310_write_byte(IST8310_REG_PDCTL, IST8310_PD_NORMAL);
	blocked_delay_ms(10);

	ist8310.last_update_time = get_sys_time_s();
}

void ist8310_semaphore_handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ist8310_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void ist8310_read_sensor(void)
{
	/* check sensor data is ready or not */
	uint8_t status = ist8310_read_byte(IST8310_REG_STAT1);
	uint8_t data_ready = status & 0x01;
	if(data_ready == 0) {
		return;
	}

	/* read sensor datas via i2c */
	uint8_t buf[6];
	ist8310_read_bytes(IST8310_REG_DATA, buf, 6);

	/* composite unscaled data */
	ist8310.mag_unscaled[0] = (((int16_t)buf[3]) << 8 | buf[2]);
	ist8310.mag_unscaled[1] = (((int16_t)buf[1]) << 8 | buf[0]);
	ist8310.mag_unscaled[2] = (((int16_t)buf[5]) << 8 | buf[4]);

	/* convert unscaled data to raw data (NED frame) */
	ist8310.mag_raw[0] = ist8310.mag_unscaled[0] * IST8310_RESOLUTION * 0.01;
	ist8310.mag_raw[1] = ist8310.mag_unscaled[1] * IST8310_RESOLUTION * 0.01;
	ist8310.mag_raw[2] = ist8310.mag_unscaled[2] * IST8310_RESOLUTION * 0.01;

	/* calculate update frequency */
	float curr_time = get_sys_time_s();
	float elapsed_time = curr_time - ist8310.last_update_time;
	ist8310.update_freq = 1.0f / elapsed_time;
	ist8310.last_update_time = curr_time;
}

void ist8310_get_raw_mag(float *mag_raw)
{
	mag_raw[0] = ist8310.mag_raw[0];
	mag_raw[1] = ist8310.mag_raw[1];
	mag_raw[2] = ist8310.mag_raw[2];
}

float ist8310_get_raw_mag_strength(void)
{
	float mx = ist8310.mag_raw[0];
	float my = ist8310.mag_raw[1];
	float mz = ist8310.mag_raw[2];

	float mag_strength;
	arm_sqrt_f32(mx*mx + my*my + mz*mz, &mag_strength);

	return mag_strength;
}

float ist8310_get_update_freq(void)
{
	return ist8310.update_freq;
}

void ist8310_task_handler(void)
{
	ist8130_init();

	while(1) {
		ist8310_read_sensor();

		/* output data rate (odr) = 50Hz
		   without data ready interrupt, it requires at least 500Hz
		   to check status registart via i2c and read the data */
		freertos_task_delay(2);
	}
}
