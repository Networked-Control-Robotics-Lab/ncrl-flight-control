#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <proj_config.h>
#include <timer.h>
#include <sys_time.h>
#include <delay.h>
#include "ncp5623c.h"
#include "i2c.h"
#include "led.h"
#include "flash.h"
#include "crc.h"
#include "spi.h"
#include "exti.h"
#include "imu.h"
#include "string.h"
#include "stm32f4xx_conf.h"
#include "uart.h"
#include <stdio.h>
#include "debug_link.h"
#include "debug_link_task.h"
#include "shell_task.h"

void init_GPIOE()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14 |GPIO_Pin_3,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void init_GPIOC()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_13 |GPIO_Pin_15,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void init_GPIOD()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_7,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_Init(GPIOD, &GPIO_InitStruct);
}
void task1(void *param)
{
	int flag = 0;
	
	imu_init();
	
	while(1) {
		if (flag == 0) {
			set_rgb_led_service_motor_lock_flag(true);
		} else if (flag == 1) {
			set_rgb_led_service_navigation_on_flag(true);
		} else if (flag == 2) {
			set_rgb_led_service_motor_lock_flag(false);
		} else if (flag == 3) {
			set_rgb_led_service_navigation_on_flag(false);
		}
		flag ++;
		flag %= 4;
		freertos_task_delay(500); //XXX: 20Hz
	}
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* driver initialization */
	flash_init();
	crc_init();
	/* rgb led */
	Init_I2C();
	
	init_GPIOC();
	init_GPIOD();
	init_GPIOE();
	GPIO_SetBits(GPIOE, GPIO_Pin_3);	//VDD_3V3_SENSORS_EN
	GPIO_SetBits(GPIOC, GPIO_Pin_2);	//MPU_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_13);	//GYRO_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_15);	//ACCEL_MAG_CS
	GPIO_SetBits(GPIOD, GPIO_Pin_7);	//BARO_CS

	uart2_init(115200);//telem
	spi1_init();       //imu
	
	timer12_init();
	timer3_init();
	
	exti15_init();     //imu ext interrupt
	enable_rgb_led_service();
	//sys_timer_blocked_delay_tick_ms(50);
	//s.bus
	uart6_init(100000);	//s.bus
	blocked_delay_ms(50);
	
	xTaskCreate(task1, "task1", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);

	rgb_led_register_task( "rgb_led_task", 512, tskIDLE_PRIORITY + 2);
	/* debug telemetry tasks */
#if (SELECT_DEBUG_TELEM == TELEM_DEBUG_LINK)
	debug_link_register_task("debug_link", 512, tskIDLE_PRIORITY + 3);
#elif (SELECT_DEBUG_TELEM == TELEM_SHELL)
	shell_register_task("shell", 1024, tskIDLE_PRIORITY + 3);
#endif

	vTaskStartScheduler();

	while(1) {
	}

	return 0;
}
