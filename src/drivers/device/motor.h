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

#if (UAV_HARDWARE == UAV_HARDWARE_AVILON) 
	#define MOTOR1 &TIM4->CCR1
	#define MOTOR2 &TIM4->CCR2
	#define MOTOR3 &TIM1->CCR4
	#define MOTOR4 &TIM1->CCR3
	#define MOTOR5 &TIM4->CCR3
	#define MOTOR6 &TIM4->CCR4
#elif (UAV_HARDWARE == UAV_HARDWARE_PIXHAWK2_4_6)
	#define MOTOR1 &TIM1->CCR4
	#define MOTOR2 &TIM1->CCR3
	#define MOTOR3 &TIM1->CCR2
	#define MOTOR4 &TIM1->CCR1
	#define MOTOR5 &TIM4->CCR2
	//#define MOTOR6 &TIM4->CCR3 //used as vins mono trigger
#endif

void set_motor_pwm_pulse(volatile uint32_t *motor, uint16_t pulse);
void set_motor_value(volatile uint32_t *motor, float percentage);
void motor_init(void);
void motor_halt(void);

void motor_thrust_test(float ch1_motor_percentage);
void reversible_thrust_test(float ch1_motor_percentage);

#endif
