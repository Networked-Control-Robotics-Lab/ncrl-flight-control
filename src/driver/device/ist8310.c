#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"

#define IST8310_ADDR 0xe

uint8_t ist8310_read_byte(uint8_t addr);
void ist8310_write_byte(uint8_t addr, uint8_t data);
void ist8310_read_bytes(uint8_t addr, uint8_t *data, int size);

SemaphoreHandle_t ist8310_semphr;

float *mag_ptr;

static uint8_t ist8310_read_who_i_am(void)
{
	uint8_t id = ist8310_read_byte(0x00);
	blocked_delay_ms(10);
	return id;
}

void ist8130_init(void)
{
	ist8310_semphr = xSemaphoreCreateBinary();

	while(ist8310_read_who_i_am() != 0x10);

	ist8310_write_byte(0x0a, 0x07); //register control1: 50Hz
	blocked_delay_ms(100);

	ist8310_write_byte(0x41, 0x24); //register average: 16
	blocked_delay_ms(100);

	ist8310_write_byte(0x42, 0xc0); //register pulse duration control: normal
	blocked_delay_ms(100);
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

	uint16_t mx_unscaled, my_unscaled, mz_unscaled;

	ist8310_read_bytes(0x03, buf, 6);

	mx_unscaled = ((uint16_t)buf[1]) << 8 | buf[0];
	my_unscaled = ((uint16_t)buf[3]) << 8 | buf[2];
	mz_unscaled = ((uint16_t)buf[5]) << 8 | buf[4];

	mag_ptr[0] = mx_unscaled; //TODO: convert scale
	mag_ptr[1] = my_unscaled;
	mag_ptr[2] = mz_unscaled;
}

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

void ist8310_task_handler(void)
{
	ist8130_init();

	while(1) {
		ist8310_read_sensor();
		blocked_delay_ms(10);
	}
}
