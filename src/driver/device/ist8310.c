#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"

#define IST8310_ADDR 0xE

SemaphoreHandle_t ist8310_semphr;

float *mag_ptr;

void ist8130_init(void)
{
	ist8310_semphr = xSemaphoreCreateBinary();
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

	sw_i2c_start();
	sw_i2c_send_byte((0xE << 1) | 0);
	//TODO: wait ack
	sw_i2c_send_byte(0x3);
	//TODO: wait ack
	sw_i2c_start();

	buf[0] = sw_i2c_read_byte();
	//TODO: wait ack
	buf[1] = sw_i2c_read_byte();
	//TODO: wait ack
	buf[2] = sw_i2c_read_byte();
	//TODO: wait ack
	buf[3] = sw_i2c_read_byte();
	//TODO: wait ack
	buf[4] = sw_i2c_read_byte();
	//TODO: wait ack
	buf[5] = sw_i2c_read_byte();
	//TODO: wait ack

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

	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();

	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_blocked_wait_ack();
	data = sw_i2c_blocked_read_byte();
	sw_i2c_blocked_nack();
	sw_i2c_blocked_stop();

	return data;
}

void ist8310_task_handler(void)
{
	//ist8130_init(); //XXX

	volatile uint8_t who_i_am;

	while(1) {
		who_i_am = ist8310_read_byte(0x00);

		blocked_delay_ms(100);
	}
}
