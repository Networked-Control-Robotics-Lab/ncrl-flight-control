#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timer.h"
#include "led.h"
#include "flight_ctl.h"
#include "sys_time.h"

extern SemaphoreHandle_t flight_ctl_semphr;

void timer12_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	/* 90MHz / (2250 * 10) = 4000Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 2250 - 1,
		.TIM_Prescaler = 10 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(TIM12, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn,
		.NVIC_IRQChannelPreemptionPriority = SYS_TIMER_ISR_PRIORITY,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM12, ENABLE);
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM12, TIM_IT_Update) == SET) {
		led_toggle(LED_R);
		TIM_ClearITPendingBit(TIM12, TIM_IT_Update);

		sys_time_update_handler();

		flight_ctl_semaphore_handler();
	}
}
