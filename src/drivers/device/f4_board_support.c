#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ist8310.h"
#include "lidar_lite.h"
#include "delay.h"
#include "ins_sensor_sync.h"
#include "proj_config.h"
#include "sw_i2c.h"
#include "timer.h"

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority);

void f4_board_init(void)
{
#if ((ENABLE_MAGNETOMETER != 0) || (ENABLE_RANGEFINDER != 0))
	sw_i2c_init();
	f4_sw_i2c_driver_register_task("sw i2c driver", 512, tskIDLE_PRIORITY + 5);
#endif

#if (ENABLE_BAROMETER != 0)
	/* barometer (ms5611) */
	spi3_init();
	ms5611_init();
#endif

	/* sensor scheduler timer */
	timer3_init();
}

void f4_sw_i2c_driver_task(void *param)
{
#if (ENABLE_MAGNETOMETER != 0)
	ist8130_init();
	freertos_task_delay(10);
#endif

#if (ENABLE_RANGEFINDER != 0)
	lidar_lite_init();
	freertos_task_delay(10);
#endif

	while(ins_sync_buffer_is_ready() == false);

	while(1) {
#if ((ENABLE_MAGNETOMETER != 0) && (ENABLE_RANGEFINDER != 0))
		ist8310_read_sensor();
		lidar_lite_read_sensor();
		lidar_lite_read_sensor();
#elif (ENABLE_MAGNETOMETER != 0)
		ist8310_read_sensor();
#elif (ENABLE_RANGEFINDER != 0)
		lidar_lite_read_sensor();
#endif
	}
}

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority)
{
	xTaskCreate(f4_sw_i2c_driver_task, task_name, stack_size, NULL, priority, NULL);
}

