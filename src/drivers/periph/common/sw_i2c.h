#ifndef __SW_I2C_H__
#define __SW_I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include "coroutine.h"

#define SW_I2C_COROUTINE_DELAY(delay_tick) \
	sw_i2c_coroutine_delay_start(delay_tick); \
	CR_YIELD(); \
	if(!sw_i2c_coroutine_delay_times_up()) {CR_RETURN;}

enum {
	SW_I2C_DO_NOTHING,
	SW_I2C_START,
	SW_I2C_STOP,
	SW_I2C_SEND_BYTE,
	SW_I2C_RECEIVE_BYTE,
	SW_I2C_ACK,
	SW_I2C_NACK,
	SW_I2C_WAIT_ACK
} SW_I2C_STATE;

void sw_i2c_init(void);
void sw_i2c_start(void);
void sw_i2c_stop(void);
void sw_i2c_ack(void);
void sw_i2c_nack(void);
int sw_i2c_wait_ack(void);
uint8_t sw_i2c_read_byte(void);
void sw_i2c_send_byte(uint8_t data);

void sw_i2c_blocked_init(void);
void sw_i2c_blocked_start(void);
void sw_i2c_blocked_stop(void);
bool sw_i2c_blocked_wait_ack(void);
void sw_i2c_blocked_ack(void);
void sw_i2c_blocked_nack(void);
uint8_t sw_i2c_blocked_read_byte(void);
void sw_i2c_blocked_send_byte(uint8_t data);

void sw_i2c_write_test(void);

#endif
