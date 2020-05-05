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

#define I2C_FREQ 400000 //400k[Hz]
#define I2C_CLOCK_PERIOD_MS ((1 / I2C_FREQ) * 1000) //[ms]

#define SW_I2C_SCL GPIOE, GPIO_Pin_0
#define SW_I2C_SDA GPIOE, GPIO_Pin_1

int i2c_state = SW_I2C_DO_NOTHING;


void sw_i2c_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* 90MHz / (1125 * 1) = 8000Hz */
	TIM_TimeBaseInitTypeDef TimeBaseInitStruct = {
		.TIM_Period = 1125 - 1,
		.TIM_Prescaler = 1 - 1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(TIM2, &TimeBaseInitStruct);

	NVIC_InitTypeDef NVIC_InitStruct = {
		.NVIC_IRQChannel = TIM2_IRQn,
		.NVIC_IRQChannelPreemptionPriority = 0,//SW_I2C_TIMER_ISR_PRIORITY,
		.NVIC_IRQChannelSubPriority = 0,
		.NVIC_IRQChannelCmd = ENABLE
	};
	NVIC_Init(&NVIC_InitStruct);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
		led_toggle(LED_R);
	}
}

void sw_i2c_config_sda_in(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_IN,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
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
		.GPIO_PuPd = GPIO_PuPd_DOWN
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

void sw_i2c_start_handler(void)
{
}

void sw_i2c_stop_handler(void)
{
}

void sw_i2c_ack_handler(void)
{
}

void sw_i2c_nack_handler(void)
{
}

void sw_i2c_byte_send_handler(void)
{
}

void sw_i2c_byte_receive_handler(void)
{
}

void sw_i2c_scl_handler(void)
{
}

void sw_i2c_sda_handler(void)
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
	}
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
}

void sw_i2c_blocked_stop(void)
{
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_low();
	sw_i2c_scl_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
}

/* wait for slave device's ack
 * ret_val: true (succeeded), false (failed)
 */
bool sw_i2c_blocked_wait_ack(void)
{
	sw_i2c_config_sda_in();
	sw_i2c_sda_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();

	uint8_t trial_time = 0;

	while(sw_i2c_sda_read()) {
		trial_time++;
		if(trial_time == 255) {
			/* failed */
			sw_i2c_blocked_stop();
			return false;
		}
	}

	sw_i2c_sda_set_low();
	return true;
}

void sw_i2c_blocked_ack(void)
{
	sw_i2c_scl_set_low();
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_low();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
}

void sw_i2c_blocked_nack(void)
{
	sw_i2c_config_sda_out();
	sw_i2c_scl_set_low();
	//sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
}

void sw_i2c_blocked_send_byte(uint8_t data)
{
	sw_i2c_config_sda_out();
	sw_i2c_scl_set_low();

	int i;
	for(i = 0; i < 8; i++) {
		if(data & 0x80) {
			sw_i2c_sda_set_high();
		} else {
			sw_i2c_sda_set_low();
		}
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);

		data <<= 1;

		sw_i2c_scl_set_high();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
		sw_i2c_scl_set_low();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	}
}

uint8_t sw_i2c_blocked_read_byte(bool do_ack)
{
	sw_i2c_config_sda_in();

	uint8_t data = 0;

	int i;
	for(i = 0; i < 8; i++) {
		sw_i2c_scl_set_low();
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
		sw_i2c_scl_set_high();

		data <<= 1;
		if(sw_i2c_sda_read()) {
			data++;
		}
		sw_i2c_delay_ms(I2C_CLOCK_PERIOD_MS);
	}

	if(do_ack == true) {
		sw_i2c_blocked_ack();
	} else {
		sw_i2c_blocked_nack();
	}

	return data;
}

void sw_i2c_write_test(void)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte(0xA0);
	sw_i2c_blocked_ack();
	sw_i2c_blocked_send_byte(0xB0);
	sw_i2c_blocked_nack();
	sw_i2c_blocked_stop();
}
