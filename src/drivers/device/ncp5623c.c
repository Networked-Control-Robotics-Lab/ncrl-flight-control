#include <stdbool.h>
#include "stm32f4xx.h"
#include "i2c.h"
#include "ncp5623c.h"

#define NCP5623C_I2C_TIMEOUT 50
#define USE_TCA 1

ncp5623c_t ncp5623c = {
	.r_val = 0.0f,
	.g_val = 0.0f,
	.b_val = 0.0f
};


void ncp5623c_led_pwm_control(float red, float green, float blue)
{
	if(ncp5623c.update_request == false) {
		ncp5623c.r_val = red;
		ncp5623c.g_val = green;
		ncp5623c.b_val = blue;
		ncp5623c.update_request = true;
	}
}

void ncp5623c_led_control(bool red_on, bool green_on, bool blue_on)
{
	if(red_on == true) {
		ncp5623c.r_val = 100.0f;
	} else {
		ncp5623c.r_val = 0.0f;
	}

	if(green_on == true) {
		ncp5623c.g_val = 100.0f;
	} else {
		ncp5623c.g_val = 0.0f;
	}

	if(blue_on == true) {
		ncp5623c.b_val = 100.0f;
	} else {
		ncp5623c.b_val = 0.0f;
	}
	ncp5623c.update_request = true;
}

void ncp5623c_led_toggle(bool red_toggle, bool green_toggle, bool blue_toggle)
{
	if(red_toggle == true) {
		ncp5623c.r_val = (ncp5623c.r_val < 50.0f) ? 100.0f : 0.0f;
	}

	if(green_toggle == true) {
		ncp5623c.g_val = (ncp5623c.g_val < 50.0f) ? 100.0f : 0.0f;
	}

	if(blue_toggle == true) {
		ncp5623c.b_val = (ncp5623c.b_val < 50.0f) ? 100.0f : 0.0f;
	}
	ncp5623c.update_request = true;
}


#if (USE_TCA == 0)
void ncp5623c_write(float red, float green, float blue)
{
	
	float timeout = NCP5623C_I2C_TIMEOUT;

	i2c_start(I2C2, 0x39 << 1, I2C_Direction_Transmitter, timeout);
	i2c_write(I2C2, 0x20 | 0x1a, timeout);
	i2c_write(I2C2, 0xF0, timeout);
	i2c_write(I2C2, 0x40 | ((uint8_t)((float)0x1f * red / 100.0f) & 0x1f), timeout);
	i2c_write(I2C2, 0xF0, timeout);
	i2c_write(I2C2, 0x60 | ((uint8_t)((float)0x1f * green / 100.0f) & 0x1f), timeout);
	i2c_write(I2C2, 0xF0,timeout);
	i2c_write(I2C2, 0x80 | ((uint8_t)((float)0x1f * blue / 100.0f) & 0x1f), timeout);
	i2c_stop(I2C2);


}

#else

void ncp5623c_write(float red, float green, float blue)
{
	float timeout = NCP5623C_I2C_TIMEOUT;

	i2c_start(I2C2, 0x55<<1|0x00, I2C_Direction_Transmitter, timeout);
	i2c_write(I2C2, 0x81 , timeout);
	i2c_write(I2C2, ((uint8_t)((float)0x0f * blue / 100.0f) & 0x0f), timeout);
	i2c_write(I2C2, 0x82 , timeout);
	i2c_write(I2C2, ((uint8_t)((float)0x0f * green / 100.0f) & 0x0f), timeout);
	i2c_write(I2C2, 0x83 , timeout);
	i2c_write(I2C2, ((uint8_t)((float)0x0f * red / 100.0f) & 0x0f), timeout);
	i2c_write(I2C2, 0x84 , timeout);
	i2c_write(I2C2, 0x03, timeout);
	i2c_stop(I2C2);
}

#endif

void ncp5623c_driver_task(void *param)
{
	while(1) {
		if(ncp5623c.update_request == true) {
			ncp5623c_write(ncp5623c.r_val, ncp5623c.g_val, ncp5623c.b_val);
			ncp5623c.update_request = false;
		}
		taskYIELD();
	}
}

void ncp5623c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                   UBaseType_t priority)
{
	xTaskCreate(ncp5623c_driver_task, task_name, stack_size, NULL, priority, NULL);
}
