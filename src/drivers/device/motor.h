#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"
#include "proj_config.h"

/* non-reversible esc pwm range*/
#define MOTOR_PULSE_MAX 20750
#define MOTOR_PULSE_MIN 10900

/* reversible esc pwm range (we use blue robotics' basic esc here) */
#define REVERSIBLE_MOTOR_PULSE_MAX  19000 //1900ms
#define REVERSIBLE_MOTOR_PULSE_STOP 15000 //1500ms
#define REVERSIBLE_MOTOR_PULSE_MIN  11000 //1100ms

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse);
void set_motor_value(volatile uint32_t *motor, float percentage);
void set_all_motor_pwm_pulse(uint16_t pulse);
void set_all_motor_value(float pulse);

void motor_init(void);
void motor_halt(void);

void motor_thrust_test(float ch1_motor_percentage);
void reversible_thrust_test(float ch1_motor_percentage);

#endif
