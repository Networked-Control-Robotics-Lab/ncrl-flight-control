#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ist8310.h"
#include "lidar_lite.h"
#include "delay.h"
#include "ins_sensor_sync.h"
#include "proj_config.h"

void f4_sw_i2c_driver_task(void *param);

SemaphoreHandle_t f4_sw_i2c_driver_semphr;

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority)
{
	f4_sw_i2c_driver_semphr = xSemaphoreCreateBinary();
	xTaskCreate(f4_sw_i2c_driver_task, task_name, stack_size, NULL, priority, NULL);
}

void f4_sw_i2c_driver_semaphore_handler(BaseType_t *higher_priority_task_woken)
{
	xSemaphoreGiveFromISR(f4_sw_i2c_driver_semphr, higher_priority_task_woken);
}

void f4_sw_i2c_driver_task(void *param)
{
#if (ENABLE_MAGNETOMETER == 1)
	ist8130_init();
	freertos_task_delay(10);
#endif

#if (ENABLE_RANGEFINDER == 1)
	lidar_lite_init();
	freertos_task_delay(10);
#endif

	while(ins_sync_buffer_is_ready() == false);

	while(1) {
#if ((ENABLE_MAGNETOMETER == 1) && (ENABLE_RANGEFINDER == 1))
		ist8310_read_sensor();
		lidar_lite_read_sensor();
		lidar_lite_read_sensor();
#elif (ENABLE_MAGNETOMETER == 1)
		ist8310_read_sensor();
#elif (ENABLE_RANGEFINDER == 1)
		lidar_lite_read_sensor();
#endif
	}
}
