#include <stdint.h>

#include "delay.h"
#include "motor.h"

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse)
{
	if(pulse < MOTOR_PULSE_MIN) {
		*motor = MOTOR_PULSE_MIN;
	} else if(pulse > MOTOR_PULSE_MAX) {
		*motor = MOTOR_PULSE_MAX;
	} else {
		*motor = pulse;
	}
}

void motor_init(void)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR5, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR6, MOTOR_PULSE_MIN);
	blocked_delay_ms(1000);
}

void motor_thrust_test(float ch1_motor_percentage)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	blocked_delay_ms(3000);

	ch1_motor_percentage /= 100.0f;
	float motor_range = (float)MOTOR_PULSE_MAX - MOTOR_PULSE_MIN;
	float motor_bias = (float)MOTOR_PULSE_MIN;
	set_motor_pwm_pulse(MOTOR1, (uint16_t)(motor_range * ch1_motor_percentage + motor_bias));
}

void esc_calibrate(void)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR5, MOTOR_PULSE_MAX);
	set_motor_pwm_pulse(MOTOR6, MOTOR_PULSE_MAX);

	blocked_delay_ms(6000);

	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR2, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR3, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR4, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR5, MOTOR_PULSE_MIN);
	set_motor_pwm_pulse(MOTOR6, MOTOR_PULSE_MIN);

	while(1);
}
