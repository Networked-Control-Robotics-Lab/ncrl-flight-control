#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "proj_config.h"
#include "timer.h"
#include "sys_time.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "led.h"
#include "flash.h"
#include "crc.h"
#include "spi.h"
#include "exti.h"
#include "sw_i2c.h"
#include "ist8310.h"
#include "optitrack.h"
#include "imu.h"
#include "pwm.h"
#include "uart.h"
#include "ncp5623c.h"
#include "ms5611.h"
#include "board_porting.h"
#include "ins_sensor_sync.h"
#include "ublox_m8n.h"

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority);
void board_init(void)
{
	/* driver initialization */
	
	flash_init();
	_crc_init();
	px4_board_gpio_config();
	i2c2_init();        //rgb controller
	uart3_init(115200); //mavlink
	uart2_init(115200); //telem
	uart6_init(100000); //s-bus

	// position sensors in pixhawk
#if (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_GPS)
	uart4_init(38400); //gps
	ublox_m8n_init();
#elif (SELECT_NAVIGATION_DEVICE1 == NAV_DEV1_USE_OPTITRACK)
	uart4_init(115200);
	optitrack_init(UAV_DEFAULT_ID); //setup tracker id for this MAV
#endif

#if (SELECT_NAVIGATION_DEVICE2 == NAV_DEV2_USE_VINS_MONO)
	//uart6_init(115200);
	//vins_mono_init(UAV_DEFAULT_ID);
#endif

	timer12_init();
	pwm_timer1_init(); //motor
	pwm_timer4_init(); //motor
	exti15_init();     //imu ext interrupt
	spi1_init();       //imu
	
#if ((ENABLE_MAGNETOMETER != 0) || (ENABLE_RANGEFINDER != 0))

#if (IST8310_I2C_USE == IST8310_I2C_USE_SW)
	sw_i2c_init();
#elif (IST8310_I2C_USE == IST8310_I2C_USE_HW)
	i2c1_init();
#endif

#endif
	timer3_init();

	blocked_delay_ms(50);


	spi1_semphare_create();
#if ((ENABLE_MAGNETOMETER != 0) || (ENABLE_RANGEFINDER != 0))
	f4_sw_i2c_driver_register_task("sw i2c driver", 512, tskIDLE_PRIORITY + 5);
#endif
	/* led driver task */
	ncp5623c_driver_register_task("led driver task", 512, tskIDLE_PRIORITY + 2);
}

/*================================*
 * ist8310 and lidar lite support *
 *================================*/
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
#if (ENABLE_MAGNETOMETER != 0)
		ist8310_read_sensor();
#endif

#if (ENABLE_RANGEFINDER != 0)
		lidar_lite_read_sensor();
#endif
		taskYIELD();
	}
}

void f4_sw_i2c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                    UBaseType_t priority)
{
	xTaskCreate(f4_sw_i2c_driver_task, task_name, stack_size, NULL, priority, NULL);
}
