#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensor_task.h"
#include "mpu6500.h"

SemaphoreHandle_t imu_semphr;

void imu_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(imu_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void task_imu(void *param)
{
	while(1) {
		while(xSemaphoreTake(imu_semphr, 9) == pdFALSE);
		mpu6500_int_handler();
		taskYIELD();
	}
}
void imu_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	imu_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_imu, task_name, stack_size, NULL, priority, NULL);
}
