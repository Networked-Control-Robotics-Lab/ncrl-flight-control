#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "freertos_config.h"
#include "delay.h"
#include "led.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "mpu6500.h"
#include "sbus_receiver.h"
#include "flight_ctl.h"
#include "sys_time.h"

extern SemaphoreHandle_t flight_ctl_semphr;

int main(void)
{
	/* freertos initialization */
	flight_ctl_semphr = xSemaphoreCreateBinary();

	/* driver initialization */
	led_init();
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(100000); //s-bus
	uart6_init(115200);
	uart7_init(115200); //gps
	timer12_init();
	pwm_timer1_init();
	pwm_timer4_init();
	spi1_init();

	led_on(LED_R);
	led_off(LED_G);
	led_off(LED_B);

	mpu6500_init();

	led_off(LED_R);
	led_off(LED_G);
	led_on(LED_B);

	xTaskCreate(task_flight_ctl, "flight control", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

	/* start freertos scheduler */
	vTaskStartScheduler();

	return 0;
}
