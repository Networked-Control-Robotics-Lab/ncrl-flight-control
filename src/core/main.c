#include <stdio.h>
#include "string.h"
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include "stm32f4xx_conf.h"
#include <proj_config.h>
#include <timer.h>
#include <sys_time.h>
#include <delay.h>
#include "ncp5623c.h"
#include "gpio.h"
#include "i2c.h"
#include "led.h"
#include "flash.h"
#include "crc.h"
#include "spi.h"
#include "exti.h"
#include "optitrack.h"
#include "imu.h"
#include "pwm.h"
#include "uart.h"
#include "debug_link.h"

#include "shell_task.h"
#include "led_task.h"
#include "mavlink_task.h"
#include "flight_ctrl_task.h"
#include "debug_link_task.h"
#include "calibration_task.h"

#include "perf.h"
#include "perf_list.h"
#include "ins_sensor_sync.h"

perf_t perf_list[] = {
	DEF_PERF(PERF_AHRS_INS, "ahrs and ins")
	DEF_PERF(PERF_CONTROLLER, "controller")
	DEF_PERF(PERF_FLIGHT_CONTROL_LOOP, "flight control loop")
	DEF_PERF(PERF_FLIGHT_CONTROL_TRIGGER_TIME, "flight control trigger time")
};

int main()
{
	perf_init(perf_list, SIZE_OF_PERF_LIST(perf_list));

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* initialize sensor synchronization buffer */
	ins_sync_buffer_init();

	/* driver initialization */
	flash_init();
	crc_init();
	/* rgb led */
#if (UAV_HARDWARE == UAV_HARDWARE_AVILON)
	led_init();
	ext_switch_init();
#elif (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	i2c2_init();
#endif

// init chip selectors gpio
#if (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	init_GPIOC();
	init_GPIOD();
	init_GPIOE();
	GPIO_SetBits(GPIOE, GPIO_Pin_3);	//VDD_3V3_SENSORS_EN
	GPIO_SetBits(GPIOC, GPIO_Pin_2);	//MPU_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_13);	//GYRO_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_15);	//ACCEL_MAG_CS
	GPIO_SetBits(GPIOD, GPIO_Pin_7);	//BARO_CS
#endif



// uart communications in avilon
#if (UAV_HARDWARE == UAV_HARDWARE_AVILON)
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(100000); //s-bus
// uart communications in pixhawk
#elif (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	uart3_init(115200); //mavlink
	uart2_init(115200);	//telem
	uart6_init(100000);	//s.bus
#endif


// position sensors in avilon
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS) && (UAV_HARDWARE == UAV_HARDWARE_AVILON)
	uart7_init(38400); //gps
	ublox_m8n_init();
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK) && (UAV_HARDWARE == UAV_HARDWARE_AVILON)
	uart4_init(115200);
	optitrack_init(UAV_DEFAULT_ID); //setup tracker id for this MAV
#endif

#if (SELECT_NAVIGATION_DEVICE2 == NAV_DEV2_USE_VINS_MONO) && (UAV_HARDWARE == UAV_HARDWARE_AVILON)

	uart6_init(115200);
	vins_mono_init(UAV_DEFAULT_ID); //TODO: tracker id is not needed
#endif

// position sensors in pixhawk
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS) && (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	/*didn't implement*/
	//uart7_init(38400); //gps
	//ublox_m8n_init();
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK) && (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	uart4_init(115200);
	optitrack_init(UAV_DEFAULT_ID); //setup tracker id for this MAV
#endif

#if (SELECT_NAVIGATION_DEVICE2 == NAV_DEV2_USE_VINS_MONO) && (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	/*didn't implement*/
	//uart6_init(115200);
	//vins_mono_init(UAV_DEFAULT_ID); //TODO: tracker id is not needed
#endif

	timer12_init();
	pwm_timer1_init(); //motor
	pwm_timer4_init(); //motor

#if (UAV_HARDWARE == UAV_HARDWARE_AVILON)
	exti10_init();     //imu ext interrupt
#elif (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	exti15_init();     //imu ext interrupt
#endif
	spi1_init();       //imu

	enable_rgb_led_service();

	//sys_timer_blocked_delay_tick_ms(50);
	//s.bus
	blocked_delay_ms(50);

#if (ENABLE_MAGNETOMETER == 1)
	/* compass (ist8310) */
	sw_i2c_init();
	ist8310_register_task("compass driver", 512, tskIDLE_PRIORITY + 5);
#endif

#if (ENABLE_BAROMETER == 1)
	/* barometer (ms5611) */
	spi3_init();
	ms5611_init();
#endif

	timer3_init();

	/* led task */
	rgb_led_register_task( "rgb_led_task", 512, tskIDLE_PRIORITY + 2);

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

	while(1) {
	}

	return 0;
}
