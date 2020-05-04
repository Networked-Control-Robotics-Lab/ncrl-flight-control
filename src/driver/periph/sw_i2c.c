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
#include "delay.h"

#define I2C_FREQ 400000 //400k[Hz]
#define I2C_CLOCK_PERIOD_MS ((1 / I2C_FREQ) * 1000) //[ms]

#define SW_I2C_SCL GPIOE, GPIO_Pin_0
#define SW_I2C_SDA GPIOE, GPIO_Pin_1

int i2c_state = SW_I2C_IDLE;

void sw_i2c_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_DOWN
	};

	GPIO_Init(GPIOE, &GPIO_InitStruct);
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

void sw_i2c_scl_handler(void)
{
}

void sw_i2c_sda_handler(void)
{
	switch(i2c_state) {
	case SW_I2C_IDLE:
		break;
	case SW_I2C_START:
		break;
	case SW_I2C_STOP:
		break;
	case SW_I2C_SEND_BIT:
		break;
	case SW_I2C_RECEIVE_BIT:
		break;
	case SW_I2C_ACK:
		break;
	case SW_I2C_NACK:
		break;
	}
}

/**************************************************************/
/* blocked i/o type i2c, should be used only for test purpose */
/**************************************************************/
void sw_i2c_start(void)
{
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_high();
	sw_i2c_scl_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_low();
}

void sw_i2c_stop(void)
{
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_low();
	sw_i2c_scl_set_low();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
}

/* wait for slave device's ack
 * ret_val: true (succeeded), false (failed)
 */
bool sw_i2c_wait_ack(void)
{
	sw_i2c_config_sda_in();
	sw_i2c_sda_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();

	uint8_t trial_time = 0;

	while(sw_i2c_sda_read()) {
		trial_time++;
		if(trial_time == 255) {
			/* failed */
			sw_i2c_stop();
			return false;
		}
	}

	sw_i2c_sda_set_low();
	return true;
}

void sw_i2c_ack(void)
{
	sw_i2c_scl_set_low();
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_low();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
}

void sw_i2c_nack(void)
{
	sw_i2c_scl_set_low();
	sw_i2c_config_sda_out();
	sw_i2c_sda_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_scl_set_high();
	blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	sw_i2c_sda_set_low();
}

void sw_i2c_send_byte(uint8_t data)
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
		blocked_delay_ms(I2C_CLOCK_PERIOD_MS);

		data <<= 1;

		sw_i2c_scl_set_high();
		blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
		sw_i2c_scl_set_low();
		blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	}
}

uint8_t sw_i2c_read_byte(bool do_ack)
{
	sw_i2c_config_sda_in();

	uint8_t data = 0;

	int i;
	for(i = 0; i < 8; i++) {
		sw_i2c_scl_set_low();
		blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
		sw_i2c_scl_set_high();

		data <<= 1;
		if(sw_i2c_sda_read()) {
			data++;
		}
		blocked_delay_ms(I2C_CLOCK_PERIOD_MS);
	}

	if(do_ack == true) {
		sw_i2c_ack();
	} else {
		sw_i2c_nack();
	}

	return data;
}
