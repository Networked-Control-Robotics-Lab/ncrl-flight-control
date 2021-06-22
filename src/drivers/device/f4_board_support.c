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

#if ((ENABLE_MAGNETOMETER == 1) && (ENABLE_RANGEFINDER == 1))
	static int curr_i2c_dev = 0;
#endif

	while(1) {
		/* trigger @ 100Hz */
		while(xSemaphoreTake(f4_sw_i2c_driver_semphr, portMAX_DELAY) == pdFALSE);

#if ((ENABLE_MAGNETOMETER == 1) && (ENABLE_RANGEFINDER == 1))
		/* XXX: currently we use software i2c to read both ist8310 and lidar lite,
		   the bandwidth is limited to let both of the sensor sampled @ 40Hz */
		if(curr_i2c_dev == 0) {
			/* is8310 takes ~6ms to read the data */
			ist8310_read_sensor();
		} else if(curr_i2c_dev > 0) {
			lidar_lite_task_handler();
		}

		curr_i2c_dev++;

		/* XXX: magic code, the executing frequency of the lidar lite sampling is
		   2 times more than the ist8310, but the update rate is limited by the 6ms
		   delay causing the result of both sensor to be sampled @ 40Hz */
		if(curr_i2c_dev > 2) {
			curr_i2c_dev = 0;
		}
#elif (ENABLE_MAGNETOMETER == 1)
		ist8310_read_sensor();
#elif (ENABLE_RANGEFINDER == 1)
		lidar_lite_task_handler();
#endif
	}
}
