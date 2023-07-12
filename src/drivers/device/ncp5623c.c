#include <stdbool.h>
#include "stm32f4xx.h"
#include "i2c.h"
#include "ncp5623c.h"

#define NCP5623C_I2C_TIMEOUT 50

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

void ncp5623c_write(float red, float green, float blue)
{
	float timeout = NCP5623C_I2C_TIMEOUT;
	i2c_start(I2C2, 0x31 << 1 | 0x00, I2C_Direction_Transmitter, timeout);

	uint8_t reg_val = ((red)?0x10:0x00) | ((green)?0x04:0x00) | ((blue)?0x01:0x00) ;

	i2c_write(I2C2, 0x00, timeout); //Enable 
	i2c_write(I2C2, 0x1C, timeout);

	i2c_write(I2C2, 0x06, timeout); //B
	i2c_write(I2C2, 0x00, timeout);

	i2c_write(I2C2, 0x04, timeout);
    i2c_write(I2C2, reg_val, timeout);

	i2c_write(I2C2, 0x07, timeout); //G
	i2c_write(I2C2, 0x00, timeout);

	i2c_write(I2C2, 0x04, timeout);
    i2c_write(I2C2, reg_val, timeout);

	i2c_write(I2C2, 0x08, timeout); //R
	i2c_write(I2C2, 0x00, timeout);

	i2c_write(I2C2, 0x04, timeout);
    i2c_write(I2C2, reg_val, timeout);

	// i2c_write(I2C2, 0x09, timeout); //
	// i2c_write(I2C2, 0x00, timeout);

	// i2c_write(I2C2, 0x04, timeout);
    // i2c_write(I2C2, reg_val, timeout);



	// i2c_write(I2C2, 0x04, timeout); 
	// i2c_write(I2C2, 0xc, timeout);

	// i2c_write(I2C2, 0x02, timeout);
	// i2c_write(I2C2, 0x3F, timeout);

	i2c_stop(I2C2);

}

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
