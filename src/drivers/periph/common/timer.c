#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timer.h"
#include "gpio.h"
#include "flight_ctrl_task.h"
#include "sys_time.h"
#include "led.h"
#include "ms5611.h"
#include "ist8310.h"
#include "proj_config.h"
#include "debug_link_task.h"
#include "dummy_sensors.h"
#include "board_init.h"

#define FLIGHT_CTRL_PRESCALER_RELOAD 1000  //400Hz
#define LED_CTRL_PRESCALER_RELOAD    16000 //25Hz

void timer12_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	/* 90MHz / (225 * 1) = 400000Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 225 - 1,
		.TIM_Prescaler = 1 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(TIM12, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn,
		.NVIC_IRQChannelPreemptionPriority = SYS_TIMER_PRIORITY,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM12, ENABLE);
}

void timer3_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 90MHz / (22500 * 10) = 400Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 22500 - 1,
		.TIM_Prescaler = 10 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(TIM3, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM3_IRQn,
		.NVIC_IRQChannelPreemptionPriority = BAROMETER_PRIORITY,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	static int flight_ctrl_cnt = FLIGHT_CTRL_PRESCALER_RELOAD;
	static int led_ctrl_cnt = LED_CTRL_PRESCALER_RELOAD;

	if(TIM_GetITStatus(TIM12, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM12, TIM_IT_Update);

		sys_time_update_handler();

		flight_ctrl_cnt--;
		if(flight_ctrl_cnt == 0) {
			flight_ctrl_cnt = FLIGHT_CTRL_PRESCALER_RELOAD;
			flight_ctrl_semaphore_handler();
		}

		led_ctrl_cnt--;
		if(led_ctrl_cnt == 0) {
			led_ctrl_cnt = LED_CTRL_PRESCALER_RELOAD;
			rgb_led_handler();
		}
	}
}

void TIM3_IRQHandler(void)
{
	/* timer3 for scheduling sensor measurements */
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		BaseType_t higher_priority_task_woken = pdFALSE;

		ms5611_driver_trigger_handler();

		//dummy_sensors_update_isr_handler(&higher_priority_task_woken);

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}
