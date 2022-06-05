#ifndef __NCP5623C_H__
#define __NCP5623C_H__
#include <stm32f4xx.h>

#define RGB_PERCENTAGE_MAX 100.0f

typedef struct {
	float r_val;
	float g_val;
	float b_val;
	bool update_request;
} ncp5623c_t;

void ncp5623c_led_pwm_control(float red, float green, float blue);   //percentage of r, g, b
void ncp5623c_led_control(bool red_on, bool green_on, bool blue_on); //on or off of r, g, b
void ncp5623c_led_toggle(bool red_toggle, bool green_toggle, bool blue_toggle);

void ncp5623c_driver_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                                   UBaseType_t priority);

#endif
