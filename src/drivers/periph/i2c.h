#ifndef __I2C_H__ 
#define __I2C_H__ 

#include <stm32f4xx.h>
void Init_I2C(void);

int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint32_t timeout);
void I2C_stop(I2C_TypeDef* I2Cx);

int I2C_write(I2C_TypeDef* I2Cx, uint8_t data, uint32_t timeout);

#endif
