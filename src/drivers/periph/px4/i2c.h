#ifndef __I2C_H__
#define __I2C_H__

#include <stm32f4xx.h>

void i2c2_init(void);

int  i2c_start(I2C_TypeDef* i2c, uint8_t address, uint8_t direction, float timeout);
void i2c_stop(I2C_TypeDef* i2c);
int  i2c_write(I2C_TypeDef* i2c, uint8_t data, float timeout);

#endif
