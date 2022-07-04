#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "freertos_config.h"
#include "delay.h"
#include "debug_link.h"
#include "shell_task.h"
#include "mavlink_task.h"
#include "flight_ctrl_task.h"
#include "debug_link_task.h"
#include "calibration_task.h"
#include "perf.h"
#include "perf_list.h"
#include "ins_sensor_sync.h"
#include "board_init.h"
#include "board_porting.h"
#include "calibration_task.h"
#include "led.h"
#include "proj_config.h"

perf_t perf_list[] = {
	DEF_PERF(PERF_AHRS_INS, "ahrs and ins")
	DEF_PERF(PERF_CONTROLLER, "controller")
	DEF_PERF(PERF_FLIGHT_CONTROL_LOOP, "flight control loop")
	DEF_PERF(PERF_FLIGHT_CONTROL_TRIGGER_TIME, "flight control trigger time")
};

int main(void)
{
	perf_init(perf_list, SIZE_OF_PERF_LIST(perf_list));

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* initialize sensor synchronization buffer */
	ins_sync_buffer_init();

	/* driver initialization */
	board_init();

	/* rgb led indicator service */
	rgb_led_service_init();

	/* flight controller task (highest priority) */
	flight_controller_register_task("flight controller", 4096, tskIDLE_PRIORITY + 6);

	/* main telemetry tasks */
#if (SELECT_MAIN_TELEM == TELEM_MAVLINK)
	mavlink_tx_register_task("mavlink publisher", 1024, tskIDLE_PRIORITY + 3);
	mavlink_rx_register_task("mavlink receiver", 2048, tskIDLE_PRIORITY + 3);
#endif

	/* debug telemetry tasks */
#if (SELECT_DEBUG_TELEM == TELEM_DEBUG_LINK)
	debug_link_register_task("debug_link", 512, tskIDLE_PRIORITY + 3);
#elif (SELECT_DEBUG_TELEM == TELEM_SHELL)
	shell_register_task("shell", 1024, tskIDLE_PRIORITY + 3);
#endif

	/* sensor calibration task
	 * inactivated by default, awakened by shell or ground station */
	calibration_register_task("calibration", 1024, tskIDLE_PRIORITY + 2);

	/* start freertos scheduler */
	vTaskStartScheduler();

	return 0;
}
