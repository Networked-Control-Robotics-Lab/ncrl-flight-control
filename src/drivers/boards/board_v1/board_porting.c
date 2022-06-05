#include "delay.h"
#include "gpio.h"
#include "motor.h"
#include "board_porting.h"

void motor_init(void)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR5, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR6, MOTOR_PULSE_MIN);
	blocked_delay_ms(100);
}

void motor_halt(void)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR5, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR6, MOTOR_PULSE_MIN);
}

void camera_trigger_gpio_on(void)
{
	gpio_on(MOTOR8);
}

void camera_trigger_gpio_off(void)
{
	gpio_off(MOTOR8);
}
