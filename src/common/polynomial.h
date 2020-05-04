#ifndef __POLYNOMIAL_H__
#define __POLYNOMIAL_H__

float calc_2th_polynomial(float *c, float t);
float calc_3th_polynomial(float *c, float t);
float calc_5th_polynomial(float *c, float t);
float calc_6th_polynomial(float *c, float t);
float calc_7th_polynomial(float *c, float t);
void copy_3th_polynomial_coefficients(float *dest, float *src);
void copy_7th_polynomial_coefficients(float *dest, float *src);
void differentiate_3th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff);
void differentiate_6th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff);
void differentiate_7th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff);

#endif
