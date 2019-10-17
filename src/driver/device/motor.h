#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"

#define MOTOR_PULSE_MAX 20000
#define MOTOR_PULSE_MIN  8000

#define MOTOR1 &TIM4->CCR1
#define MOTOR2 &TIM4->CCR2
#define MOTOR3 &TIM1->CCR4
#define MOTOR4 &TIM1->CCR3
#define MOTOR5 &TIM4->CCR3
#define MOTOR6 &TIM4->CCR4

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse);
void motor_init();

void esc_calibrate();

#endif
