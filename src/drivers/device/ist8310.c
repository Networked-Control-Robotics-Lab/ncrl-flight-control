#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"
#include "ist8310.h"

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
}

void ist8310_semaphore_handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ist8310_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void ist8310_read_sensor(void)
{
	uint8_t buf[6];

	ist8310_read_bytes(IST8310_REG_DATA, buf, 6);

	ist8310.mag_unscaled[0] = (((int16_t)buf[1]) << 8 | buf[0]);
	ist8310.mag_unscaled[1] = (((int16_t)buf[3]) << 8 | buf[2]);
	ist8310.mag_unscaled[2] = (((int16_t)buf[5]) << 8 | buf[4]);

	ist8310.mag_raw[0] = ist8310.mag_unscaled[0] * IST8310_RESOLUTION_3MG;
	ist8310.mag_raw[1] = ist8310.mag_unscaled[1] * IST8310_RESOLUTION_3MG;
	ist8310.mag_raw[2] = ist8310.mag_unscaled[2] * IST8310_RESOLUTION_3MG;
}

void ist8310_get_raw_mag(float *mag_raw)
{
	mag_raw[0] = ist8310.mag_raw[0];
	mag_raw[1] = ist8310.mag_raw[1];
	mag_raw[2] = ist8310.mag_raw[2];
}

void ist8310_task_handler(void)
{
	ist8130_init();

	while(1) {
		ist8310_read_sensor();
		blocked_delay_ms(10);
	}
}
