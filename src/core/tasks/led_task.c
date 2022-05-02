#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "led.h"
#include "led_task.h"
#include "proj_config.h"

SemaphoreHandle_t rgb_led_semphr;

void rgb_led_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(rgb_led_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void task_rgb_led(void *param)
{	
	while(1) {
		while(xSemaphoreTake(rgb_led_semphr, 9) == pdFALSE);
		rgb_led_handler();
	}
}
void rgb_led_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	rgb_led_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_rgb_led, task_name, stack_size, NULL, priority, NULL);
}
