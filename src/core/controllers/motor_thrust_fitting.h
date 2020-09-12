#ifndef __MOTOR_TRHUST_FITTING_H__
#define __MOTOR_THRUST_FITTING_H__

void set_motor_max_thrust(float max);
float set_motor_cmd_to_thrust_coeff(float c1, float c2, float c3, float c4, float c5, float c6);
float set_motor_thrust_to_cmd_coeff(float c1, float c2, float c3, float c4, float c5, float c6);

float convert_motor_cmd_to_thrust(float percentage);
float convert_motor_thrust_to_cmd(float thrust);

#endif
