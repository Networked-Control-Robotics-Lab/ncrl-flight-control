#ifndef __SW_I2C_H__
#define __SW_I2C_H__

enum {
	SW_I2C_DO_NOTHING,
	SW_I2C_START,
	SW_I2C_STOP,
	SW_I2C_SEND_BYTE,
	SW_I2C_RECEIVE_BYTE,
	SW_I2C_ACK,
	SW_I2C_NACK
} SW_I2C_STATE;

void sw_i2c_init(void);
void sw_i2c_write_test(void);

#endif
