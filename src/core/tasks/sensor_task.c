#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensor_task.h"
#include "mpu6500.h"
#include "ms5611.h"
#include "spi.h"

SemaphoreHandle_t imu_semphr;
SemaphoreHandle_t baro_semphr;

void imu_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(imu_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void baro_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(baro_semphr, &xHigherPriorityTaskWoken);
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
void task_baro(void *param)
{
	while(1) {
		while(xSemaphoreTake(baro_semphr, 9) == pdFALSE);
		ms5611_driver_trigger_handler();
		//mpu6500_int_handler();
		taskYIELD();
	}
}
void imu_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	imu_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_imu, task_name, stack_size, NULL, priority, NULL);
}
void baro_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                     UBaseType_t priority)
{
	baro_semphr = xSemaphoreCreateBinary();
	xTaskCreate(task_baro, task_name, stack_size, NULL, priority, NULL);
}
