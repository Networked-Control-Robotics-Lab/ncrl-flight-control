#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"

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

void ist8310_task_handler(void)
{
	//ist8130_init(); //XXX

	while(1) {
		sw_i2c_start();
		sw_i2c_send_byte(0xA0);
		sw_i2c_ack();
		sw_i2c_send_byte(0xB0);
		sw_i2c_nack();
		sw_i2c_stop();

		freertos_task_delay(100);
	}
}
