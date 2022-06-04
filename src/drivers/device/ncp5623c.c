#include <stm32f4xx.h>
#include"i2c.h"

#define NCP5623C_I2C_TIMEOUT 50

void rgb_light(float red, float green, float blue)
{
	uint32_t timeout = NCP5623C_I2C_TIMEOUT;

	i2c_start(I2C2, 0x39 << 1, I2C_Direction_Transmitter, timeout);
	i2c_write(I2C2, 0x20 | 0x1f, timeout);
	i2c_write(I2C2, 0x70, timeout);
	i2c_write(I2C2, 0x40 | ((uint8_t)((float)0x1f * red / 100.0f) & 0x1f), timeout);
	i2c_write(I2C2, 0x70, timeout);
	i2c_write(I2C2, 0x60 | ((uint8_t)((float)0x1f * green / 100.0f) & 0x1f), timeout);
	i2c_write(I2C2, 0x70,timeout);
	i2c_write(I2C2, 0x80 | ((uint8_t)((float)0x1f * blue / 100.0f ) & 0x1f), timeout);
	i2c_stop(I2C2);
}
