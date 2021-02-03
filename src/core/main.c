#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "freertos_config.h"
#include "delay.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "exti.h"
#include "mpu6500.h"
#include "sbus_radio.h"
#include "optitrack.h"
#include "sys_time.h"
#include "motor.h"
#include "debug_link.h"
#include "multirotor_pid_ctrl.h"
#include "flight_ctrl_task.h"
#include "shell_task.h"
#include "proj_config.h"
#include "mavlink_task.h"
#include "debug_link_task.h"
#include "perf.h"
#include "perf_list.h"
#include "sw_i2c.h"
#include "crc.h"
#include "ublox_m8n.h"
#include "calibration_task.h"
#include "flash.h"
#include "ms5611.h"
#include "ist8310.h"

perf_t perf_list[] = {
	DEF_PERF(PERF_AHRS, "ahrs")
	DEF_PERF(PERF_CONTROLLER, "controller")
	DEF_PERF(PERF_FLIGHT_CONTROL_LOOP, "flight control loop")
	DEF_PERF(PERF_FLIGHT_CONTROL_TRIGGER_TIME, "flight control trigger time")
};

int main(void)
{
	perf_init(perf_list, SIZE_OF_PERF_LIST(perf_list));

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* driver initialization */
	flash_init();
	crc_init();
	led_init();
	ext_switch_init();
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(100000); //s-bus
	uart6_init(115200);

#if (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_GPS)
	uart7_init(38400); //gps
	ublox_m8n_init();
#elif (SELECT_POSITION_SENSOR == POSITION_SENSOR_USE_OPTITRACK)
	uart7_init(115200); //optitrack
	optitrack_init(UAV_ID); //setup tracker id for this MAV
#endif

	timer12_init();    //system timer and flight controller timer
	pwm_timer1_init(); //motor
	pwm_timer4_init(); //motor
	exti10_init();     //imu ext interrupt
	spi1_init();       //imu

	blocked_delay_ms(50);

#if (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_COMPASS)
	/* compass (ist8310) */
	sw_i2c_init();
	ist8310_register_task("compass driver", 512, tskIDLE_PRIORITY + 5);
#endif

#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	/* barometer (ms5611) */
	spi3_init();
	ms5611_init();
#endif

	timer3_init();

	/* flight controller task (highest priority) */
	flight_controller_register_task("flight controller", 4096, tskIDLE_PRIORITY + 6);

	/* main telemetry tasks */
#if (SELECT_TELEM == TELEM_MAVLINK)
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
