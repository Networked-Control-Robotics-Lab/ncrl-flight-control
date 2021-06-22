/* software i2c driver */

/* operations defined by i2c protocol:
 * idle: sda held low, scl passive pullup (high)
 * start: sda falled down (high to low), scl passive pull up (high)
 * stop: sda rised up (low to high), scl passive pull up (high)
 * ack: sda held low by "receiver" after scl falls
 * nack: driven high by "receiver" after scl falls
 * send bit: "sender" set bit after scl falls
 * receive bit: "receiver" capture bit after scl rises
 * check: https://en.wikipedia.org/wiki/I%C2%B2C
 */

#include <stdbool.h>
#include "stm32f4xx.h"
#include "sw_i2c.h"
#include "sys_time.h"
#include "isr.h"
#include "gpio.h"
#include "coroutine.h"
#include "delay.h"

#define I2C_FREQ 200000.0 //100k[Hz], XXX:change to 400k later
#define I2C_CLOCK_PERIOD_MS ((1.0 / I2C_FREQ) * 1000.0) //[ms]

#define SW_I2C_SCL GPIOE, GPIO_Pin_0
#define SW_I2C_SDA GPIOE, GPIO_Pin_1

void sw_i2c_handler(void);

SemaphoreHandle_t sw_i2c_semphr;

volatile int i2c_state = SW_I2C_DO_NOTHING;
float coroutine_delay_start_time = 0;
float coroutine_delay_time = 0;
uint8_t sw_i2c_recpt_data;
uint8_t sw_i2c_send_data;
volatile int i2c_rw_bit_index;

void sw_i2c_init(void)
{
	sw_i2c_semphr = xSemaphoreCreateBinary();
	xSemaphoreGive(sw_i2c_semphr);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* 90MHz / (900 * 1) = 100kHz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 900 - 1,
		.TIM_Prescaler = 1 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(TIM2, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM2_IRQn,
		.NVIC_IRQChannelPreemptionPriority = SW_I2C_TIMER_ISR_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void sw_i2c_timer_enable(void)
{
	TIM_Cmd(TIM2, ENABLE);
}

void sw_i2c_timer_disable(void)
{
	TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
		sw_i2c_handler();
	}
}

void sw_i2c_config_sda_in(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_IN,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void sw_i2c_config_sda_out(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void sw_i2c_scl_set_high(void)
{
	GPIO_SetBits(SW_I2C_SCL);
}

void sw_i2c_scl_set_low(void)
{
	GPIO_ResetBits(SW_I2C_SCL);
}

void sw_i2c_sda_set_high(void)
{
	GPIO_SetBits(SW_I2C_SDA);
}

void sw_i2c_sda_set_low(void)
{
	GPIO_ResetBits(SW_I2C_SDA);
}

uint8_t sw_i2c_sda_read(void)
{
	return GPIO_ReadInputDataBit(SW_I2C_SDA);
}

void sw_i2c_coroutine_delay_start(float delay_ms)
{
	coroutine_delay_start_time = get_sys_time_ms();
	coroutine_delay_time = delay_ms;
}

bool sw_i2c_coroutine_delay_times_up(void)
{
	float curr_time = get_sys_time_ms();
	float elapsed_time = curr_time - coroutine_delay_start_time;
	if(elapsed_time >= coroutine_delay_time) {
		return true;
	} else {
		return false;
	}
}

void sw_i2c_start_handler(void)
{
	CR_START();

	sw_i2c_config_sda_out();
	sw_i2c_sda_set_high();
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_state = SW_I2C_DO_NOTHING;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);

	CR_END();
}

void sw_i2c_stop_handler(void)
{
	CR_START();

	sw_i2c_config_sda_out();
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_state = SW_I2C_DO_NOTHING;

	sw_i2c_timer_disable();

	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);

	CR_END();
}

void sw_i2c_ack_handler(void)
{
	CR_START();

	sw_i2c_config_sda_out();
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_state = SW_I2C_DO_NOTHING;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);

	CR_END();
}

void sw_i2c_nack_handler(void)
{
	CR_START();

	sw_i2c_config_sda_out();
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_state = SW_I2C_DO_NOTHING;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);

	CR_END();
}

void sw_i2c_byte_send_handler(void)
{
	CR_START();

	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	if(sw_i2c_send_data & 0x80) {
		sw_i2c_sda_set_high();
	} else {
		sw_i2c_sda_set_low();
	}
	sw_i2c_send_data <<= 1;
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_rw_bit_index++;
	if(i2c_rw_bit_index >= 8) {
		sw_i2c_scl_set_low();

		i2c_state = SW_I2C_DO_NOTHING;

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);
	}

	CR_END();
}

void sw_i2c_byte_receive_handler(void)
{
	CR_START();

	sw_i2c_recpt_data <<= 1;

	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	if(sw_i2c_sda_read()) {
		sw_i2c_recpt_data |= 0x01;
	}

	i2c_rw_bit_index++;
	if(i2c_rw_bit_index >= 8) {
		sw_i2c_scl_set_low();

		i2c_state = SW_I2C_DO_NOTHING;

		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);
	}

	CR_END();
}

void sw_i2c_wait_ack_handler(void)
{
	CR_START();

	sw_i2c_config_sda_in();

	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	float start_time = get_sys_time_ms();
	float curr_time;
	float elapsed_time;
	while(sw_i2c_sda_read()) {
		curr_time = get_sys_time_ms();
		elapsed_time = curr_time - start_time;
		if(elapsed_time >= I2C_CLOCK_PERIOD_MS) {
			/* failed */
			break;
		}
	}

	sw_i2c_scl_set_low();
	SW_I2C_COROUTINE_DELAY(I2C_CLOCK_PERIOD_MS);

	i2c_state = SW_I2C_DO_NOTHING;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(sw_i2c_semphr, &higher_priority_task_woken);

	CR_END();
}

void sw_i2c_handler(void)
{
	switch(i2c_state) {
	case SW_I2C_DO_NOTHING:
		break;
	case SW_I2C_START:
		sw_i2c_start_handler();
		break;
	case SW_I2C_STOP:
		sw_i2c_stop_handler();
		break;
	case SW_I2C_SEND_BYTE:
		sw_i2c_byte_send_handler();
		break;
	case SW_I2C_RECEIVE_BYTE:
		sw_i2c_byte_receive_handler();
		break;
	case SW_I2C_ACK:
		sw_i2c_ack_handler();
		break;
	case SW_I2C_NACK:
		sw_i2c_nack_handler();
		break;
	case SW_I2C_WAIT_ACK:
		sw_i2c_wait_ack_handler();
		break;
	}
}

void sw_i2c_start(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	sw_i2c_timer_enable();
	i2c_state = SW_I2C_START;
}

void sw_i2c_stop(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	i2c_state = SW_I2C_STOP;
}

void sw_i2c_ack(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	i2c_state = SW_I2C_ACK;
}

void sw_i2c_nack(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	i2c_state = SW_I2C_NACK;
}

void sw_i2c_wait_ack(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	i2c_state = SW_I2C_WAIT_ACK;
}

uint8_t sw_i2c_read_byte(void)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	sw_i2c_config_sda_in();
	sw_i2c_sda_set_high();

	i2c_rw_bit_index = 0;
	i2c_state = SW_I2C_RECEIVE_BYTE;

	return sw_i2c_recpt_data;
}

void sw_i2c_send_byte(uint8_t data)
{
	while(xSemaphoreTake(sw_i2c_semphr, portMAX_DELAY) != pdTRUE);

	i2c_rw_bit_index = 0;
	sw_i2c_send_data = data;

	sw_i2c_config_sda_out();

	i2c_state = SW_I2C_SEND_BYTE;
}

/**************************************************************/
/* blocked i/o type i2c, should be used only for test purpose */
/**************************************************************/
void sw_i2c_delay_ms(float delay_ms)
{
	float start_time = get_sys_time_ms();

	while(1) {
		float curr_time = get_sys_time_ms();
		float elapsed_time = curr_time - start_time;
		if(elapsed_time >= delay_ms) {
			return;
		}
	}
}

void sw_i2c_blocked_start(void)
{
	sw_i2c_config_sda_out();

	sw_i2c_sda_set_high();
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
}

void sw_i2c_blocked_stop(void)
{
	sw_i2c_config_sda_out();

	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
}

/* wait for slave device's ack
 * ret_val: true (succeeded), false (failed)
 */
bool sw_i2c_blocked_wait_ack(void)
{
	sw_i2c_config_sda_in();

	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

	float start_time = get_sys_time_ms();
	float curr_time;
	float elapsed_time;
	while(sw_i2c_sda_read()) {
		curr_time = get_sys_time_ms();
		elapsed_time = curr_time - start_time;
		if(elapsed_time >= I2C_CLOCK_PERIOD_MS) {
			/* failed */
			sw_i2c_scl_set_low();
			return false;
		}
	}

	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

	return true;
}

void sw_i2c_blocked_ack(void)
{
	sw_i2c_config_sda_out();

	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
}

void sw_i2c_blocked_nack(void)
{
	sw_i2c_config_sda_out();

	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
}

void sw_i2c_blocked_send_byte(uint8_t data)
{
	sw_i2c_config_sda_out();

	int i;
	for(i = 0; i < 8; i++) {
		sw_i2c_scl_set_low();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

		if(data & 0x80) {
			sw_i2c_sda_set_high();
		} else {
			sw_i2c_sda_set_low();
		}
		data <<= 1;

		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

		sw_i2c_scl_set_high();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	}
	sw_i2c_scl_set_low();
}

uint8_t sw_i2c_blocked_read_byte(void)
{
	sw_i2c_config_sda_in();

	sw_i2c_sda_set_high();

	uint8_t data = 0;
	int i;
	for(i = 0; i < 8; i++) {
		data <<= 1;

		sw_i2c_scl_set_low();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
		sw_i2c_scl_set_high();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

		if(sw_i2c_sda_read()) {
			data |= 0x01;
		}
	}
	sw_i2c_scl_set_low();

	return data;
}

void sw_i2c_write_test(void)
{
	while(1) {
#if 0
		sw_i2c_blocked_start();
		sw_i2c_blocked_send_byte(0xA0);
		sw_i2c_blocked_ack();
		sw_i2c_blocked_send_byte(0xB0);
		sw_i2c_blocked_nack();
		sw_i2c_blocked_stop();
#endif

		sw_i2c_start();
		sw_i2c_send_byte(0xA0);
		sw_i2c_ack();
		sw_i2c_send_byte(0xB0);
		sw_i2c_nack();
		sw_i2c_stop();

		blocked_delay_ms(5);
	}
}
