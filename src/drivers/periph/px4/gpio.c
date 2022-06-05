#include "stm32f4xx.h"
#include "gpio.h"

void px4_board_gpio_config(void)
{
	/* rcc initialization */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* gpio initialization */
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType =GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_13 |GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_3;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* configure gpio state for the px4 board */
	GPIO_SetBits(GPIOE, GPIO_Pin_3);  //VDD_3V3_SENSORS_EN
	GPIO_SetBits(GPIOC, GPIO_Pin_2);  //MPU_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_13); //GYRO_CS
	GPIO_SetBits(GPIOC, GPIO_Pin_15); //ACCEL_MAG_CS
	GPIO_SetBits(GPIOD, GPIO_Pin_7);  //BARO_CS
}
