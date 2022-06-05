#include <stdint.h>
#include "delay.h"
#include "motor.h"
#include "bound.h"
#include "board_porting.h"

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

/* input range: 0~1 (since 1% means 0.01) */
void set_motor_value(volatile uint32_t *motor, float percentage)
{
	bound_float(&percentage, 1.0f, 0.0f);

	float pwm_val = REVERSIBLE_MOTOR_PULSE_MIN;

	pwm_val = (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN) * percentage +
	          MOTOR_PULSE_MIN;

	*motor = (uint32_t)pwm_val;
}


/* input range: -100~100 [%] */
void set_reversible_motor_value(volatile uint32_t *motor, float percentage)
{
	bound_float(&percentage, 100.0f, -100.0f);

	float scale = percentage / 100.0f;
	float pwm_val = REVERSIBLE_MOTOR_PULSE_STOP;

	if(percentage <= 0) {
		pwm_val = (REVERSIBLE_MOTOR_PULSE_STOP - REVERSIBLE_MOTOR_PULSE_MIN) * scale +
		          REVERSIBLE_MOTOR_PULSE_STOP;
	} else if(percentage > 0) {
		pwm_val = (REVERSIBLE_MOTOR_PULSE_MAX - REVERSIBLE_MOTOR_PULSE_STOP) * scale +
		          REVERSIBLE_MOTOR_PULSE_STOP;
	}

	*motor = (uint32_t)pwm_val;
}

void motor_thrust_test(float ch1_motor_percentage)
{
	set_motor_pwm_pulse(MOTOR1, MOTOR_PULSE_MIN);
	blocked_delay_ms(3000);

	ch1_motor_percentage /= 100.0f;
	float motor_range = (float)MOTOR_PULSE_MAX - MOTOR_PULSE_MIN;
	float motor_bias = (float)MOTOR_PULSE_MIN;
	set_motor_pwm_pulse(MOTOR1, (uint16_t)(motor_range * ch1_motor_percentage + motor_bias));

	while(1);
}

void reversible_thrust_test(float ch1_motor_percentage)
{
	set_reversible_motor_value(MOTOR1, 0);
	blocked_delay_ms(3000);

	set_reversible_motor_value(MOTOR1, ch1_motor_percentage);
	while(1);
}
