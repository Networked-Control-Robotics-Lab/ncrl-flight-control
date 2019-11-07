#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "exti.h"
#include "isr.h"
#include "led.h"
#include "mpu6500.h"

void exti10_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Mode = GPIO_Mode_IN,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_Pin = GPIO_Pin_10,
		.GPIO_PuPd = GPIO_PuPd_DOWN,
	};
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
	EXTI_InitTypeDef EXTI_InitStruct = {
		.EXTI_Line = EXTI_Line10,
		.EXTI_LineCmd = ENABLE,
		.EXTI_Mode = EXTI_Mode_Interrupt,
		.EXTI_Trigger = EXTI_Trigger_Rising_Falling,
	};
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = EXTI15_10_IRQn,
		.NVIC_IRQChannelPreemptionPriority = IMU_EXTI_ISR_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE,
	};
	NVIC_Init(&NVIC_InitStruct);
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) == SET) {
		led_toggle(LED_G);
		mpu6500_int_handler();
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}
