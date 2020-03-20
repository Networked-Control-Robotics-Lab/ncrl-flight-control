#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timer.h"
#include "led.h"
#include "fc_task.h"
#include "sys_time.h"

#define FLIGHT_CTL_PRESCALER_RELOAD 1000

extern SemaphoreHandle_t flight_ctl_semphr;

void timer12_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	/* 90MHz / (225 * 10) = 40000Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 225 - 1,
		.TIM_Prescaler = 1 - 1,
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
	static int flight_ctl_cnt = FLIGHT_CTL_PRESCALER_RELOAD;
	if(TIM_GetITStatus(TIM12, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM12, TIM_IT_Update);

		sys_time_update_handler();

		if((flight_ctl_cnt--) == 0) {
			flight_ctl_cnt = FLIGHT_CTL_PRESCALER_RELOAD;
			flight_ctl_semaphore_handler();
		}
	}
}
