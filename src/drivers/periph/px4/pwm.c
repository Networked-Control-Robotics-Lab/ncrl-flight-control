#include "stm32f4xx.h"
#include "proj_config.h"

/*
 * m1: pe14 (timer1 channel4)
 * m2: pe13 (timer1 channel3)
 * m3: pe11 (timer1 channel2)
 * m4: pe9  (timer1 channel1)
 * m5: pd13 (timer4 channel2)
 * m6: pd14 (timer4 channel3), currently assigned to send camera trigger pulse
 */

void pwm_timer1_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* 180MHz / (25000 * 18) = 400Hz = 0.0025s */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
		.TIM_Period = 25000 - 1,
		.TIM_Prescaler = 18 - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 0,
	};

	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);

	TIM_Cmd(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void pwm_timer4_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin =  GPIO_Pin_13,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* motor6: vins mono camera triggering (gpio mode) */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* 90MHz / (25000 * 9) = 400Hz = 0.0025s */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
		.TIM_Period = 25000 - 1,
		.TIM_Prescaler = 9 - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_Pulse = 0,
	};

	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	//TIM_OC3Init(TIM4, &TIM_OCInitStruct);

	TIM_Cmd(TIM4, ENABLE);
}
