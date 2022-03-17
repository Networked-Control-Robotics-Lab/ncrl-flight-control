#include <stm32f4xx.h>
#include"i2c.h"

#define NCP5623C_I2C_TIMEOUT 50

void rgb_light(float red, float green, float blue)
{
	uint32_t timeout = NCP5623C_I2C_TIMEOUT;
	I2C_start(I2C2, 0x39<<1, I2C_Direction_Transmitter, 50); // start a transmission in Master transmitter mode
	I2C_write(I2C2,0x20|0x1f,timeout );
	I2C_write(I2C2,0x70,timeout );
	I2C_write(I2C2,0x40|((uint8_t)((float)0x1f * red/100.0f )&0x1f),timeout);
	I2C_write(I2C2,0x70,timeout);
	I2C_write(I2C2,0x60|((uint8_t)((float)0x1f * green/100.0f )&0x1f),timeout);
	I2C_write(I2C2,0x70,timeout);
	I2C_write(I2C2,0x80|((uint8_t)((float)0x1f * blue/100.0f )&0x1f),timeout);
	I2C_stop(I2C2);
}
