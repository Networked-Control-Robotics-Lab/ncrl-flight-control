#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"

void ist8310_task_handler(void)
{
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
