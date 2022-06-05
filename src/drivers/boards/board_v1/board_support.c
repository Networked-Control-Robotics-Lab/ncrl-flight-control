#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "proj_config.h"
#include "timer.h"
#include "sys_time.h"
#include "delay.h"
#include "gpio.h"
#include "led.h"
#include "flash.h"
#include "crc.h"
#include "spi.h"
#include "exti.h"
#include "optitrack.h"
#include "imu.h"
#include "pwm.h"
#include "uart.h"
#include "vins_mono.h"
#include "ublox_m8n.h"

void board_init(void)
{
	/* driver initialization */
	flash_init();
	crc_init();
	led_init();
	ext_switch_init();
	uart1_init(115200);
	uart3_init(115200); //telem
	uart4_init(100000); //s-bus

#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS)
	uart7_init(38400); //gps
	ublox_m8n_init();
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	uart7_init(115200);
	optitrack_init(UAV_DEFAULT_ID); //setup tracker id for this MAV
#endif

#if (SELECT_NAVIGATION_DEVICE2 == NAV_DEV2_USE_VINS_MONO)
	uart6_init(115200);
	vins_mono_init(UAV_DEFAULT_ID);
#endif

	timer12_init();    //system timer and flight controller timer
	pwm_timer1_init(); //motor
	pwm_timer4_init(); //motor
	exti10_init();     //imu ext interrupt
	spi1_init();       //imu

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
}
