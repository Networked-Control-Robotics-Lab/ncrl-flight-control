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

#define FLIGHT_CTL_PRESCALER_RELOAD      1000  //400Hz
#define LED_CTRL_PRESCALER_RELOAD        16000 //25Hz
#define COMPASS_PRESCALER_RELOAD         8     //50Hz
#define BAROMETER_PRESCALER_RELOAD       8     //100Hz

#if (DEBUG_LINK_PUBLISH_RATE == DEBUG_LINK_PUBLISH_100Hz)
#define DEBUG_LINK_TASK_PRESCALER_RELOAD 4
#elif (DEBUG_LINK_PUBLISH_RATE == DEBUG_LINK_PUBLISH_20Hz)
#define DEBUG_LINK_TASK_PRESCALER_RELOAD 20
#endif

extern SemaphoreHandle_t flight_ctl_semphr;

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
		.NVIC_IRQChannelPreemptionPriority = SYS_TIMER_ISR_PRIORITY,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM12, ENABLE);
}

void timer3_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 90MHz / (225000 * 10) = 400Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 22500 - 1,
		.TIM_Prescaler = 10 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(TIM3, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM3_IRQn,
		.NVIC_IRQChannelPreemptionPriority = BAROMETER_ISR_PRIORITY,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	static int flight_ctrl_cnt = FLIGHT_CTL_PRESCALER_RELOAD;
	//static int led_ctrl_cnt = LED_CTRL_PRESCALER_RELOAD;

	if(TIM_GetITStatus(TIM12, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM12, TIM_IT_Update);

		sys_time_update_handler();

		flight_ctrl_cnt--;
		if(flight_ctrl_cnt == 0) {
			flight_ctrl_cnt = FLIGHT_CTL_PRESCALER_RELOAD;
			flight_ctrl_semaphore_handler();
		}

#if 0
		led_ctrl_cnt--;
		if(led_ctrl_cnt == 0) {
			led_ctrl_cnt = LED_CTRL_PRESCALER_RELOAD;
			rgb_led_handler();
		}
#endif
	}
}

void TIM3_IRQHandler(void)
{
#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
	static int barometer_cnt = BAROMETER_PRESCALER_RELOAD;
#endif

#if (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_COMPASS)
	static int compass_cnt = COMPASS_PRESCALER_RELOAD;
#endif

#if (SELECT_TELEM == TELEM_DEBUG_LINK)
	static int debug_link_task_cnt = DEBUG_LINK_TASK_PRESCALER_RELOAD;
#endif

	/* trigger ms5611 driver task (400Hz) */
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		BaseType_t higher_priority_task_woken = pdFALSE;

#if (SELECT_HEIGHT_SENSOR == HEIGHT_SENSOR_USE_BAROMETER)
		/* barometer */
		barometer_cnt--;
		if(barometer_cnt == 0) {
			barometer_cnt = BAROMETER_PRESCALER_RELOAD;
			ms5611_driver_semaphore_handler(&higher_priority_task_woken);
		}
#endif

#if (SELECT_HEADING_SENSOR == HEADING_SENSOR_USE_COMPASS)
		/* compass */
		compass_cnt--;
		if(compass_cnt == 0) {
			compass_cnt = COMPASS_PRESCALER_RELOAD;
			ist8310_semaphore_handler(&higher_priority_task_woken);
		}
#endif

#if (SELECT_TELEM == TELEM_DEBUG_LINK)
		debug_link_task_cnt--;
		if(debug_link_task_cnt == 0) {
			debug_link_task_cnt = DEBUG_LINK_TASK_PRESCALER_RELOAD;
			debug_link_task_semaphore_handler();
		}
#endif

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		portEND_SWITCHING_ISR(higher_priority_task_woken);
	}
}
