#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "led.h"
#include "sbus_receiver.h"
#include "flight_ctl.h"

SemaphoreHandle_t flight_ctl_semphr;

void flight_ctl_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(flight_ctl_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void task_flight_ctl(void *param)
{
	while(1) {
		if(xSemaphoreTake(flight_ctl_semphr, 1) == pdFALSE) {
			continue;
		}

		led_toggle(LED_B);

		radio_t rc;
		read_rc_info(&rc);
		debug_print_rc_info(&rc);

		blocked_delay_ms(5);

		taskYIELD();
	}
}
