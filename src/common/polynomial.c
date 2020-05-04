float calc_2th_polynomial(float *c, float t)
{
	float t_powers[3];
	t_powers[0] = 1;
	float ret_poly = c[0] * t_powers[0];

	for(int i = 1; i < 3; i++) {
		t_powers[i] = t_powers[i - 1] * t;
		ret_poly += c[i] * t_powers[i];
	}

	return ret_poly;
}

float calc_3th_polynomial(float *c, float t)
{
	float t_powers[4];
	t_powers[0] = 1;
	float ret_poly = c[0] * t_powers[0];

	for(int i = 1; i < 4; i++) {
		t_powers[i] = t_powers[i - 1] * t;
		ret_poly += c[i] * t_powers[i];
	}

	return ret_poly;
}

float calc_5th_polynomial(float *c, float t)
{
	float t_powers[6];
	t_powers[0] = 1;
	float ret_poly = c[0] * t_powers[0];

	for(int i = 1; i < 6; i++) {
		t_powers[i] = t_powers[i - 1] * t;
		ret_poly += c[i] * t_powers[i];
	}

	return ret_poly;
}

float calc_6th_polynomial(float *c, float t)
{
	float t_powers[7];
	t_powers[0] = 1;
	float ret_poly = c[0] * t_powers[0];

	for(int i = 1; i < 7; i++) {
		t_powers[i] = t_powers[i - 1] * t;
		ret_poly += c[i] * t_powers[i];
	}

	return ret_poly;
}

float calc_7th_polynomial(float *c, float t)
{
	float t_powers[8];
	t_powers[0] = 1;
	float ret_poly = c[0] * t_powers[0];

	for(int i = 1; i < 8; i++) {
		t_powers[i] = t_powers[i - 1] * t;
		ret_poly += c[i] * t_powers[i];
	}

	return ret_poly;
}

void copy_3th_polynomial_coefficients(float *dest, float *src)
{
	dest[0] = src[0];
	dest[1] = src[1];
	dest[2] = src[2];
	dest[3] = src[3];
}

void copy_7th_polynomial_coefficients(float *dest, float *src)
{
	dest[0] = src[0];
	dest[1] = src[1];
	dest[2] = src[2];
	dest[3] = src[3];
	dest[4] = src[4];
	dest[5] = src[5];
	dest[6] = src[6];
	dest[7] = src[7];
}

/*
 * input: 3th order polynomial's coefficients
 * output: 2th order polynomial's coefficients
 */
void differentiate_3th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff)
{
	const float d0 = 1.0f;
	const float d1 = 1.0f / 2.0f;
	const float d2 = 1.0f / 3.0f;
	vel_traj_coeff[0] = d0 * pos_traj_coeff[1];
	vel_traj_coeff[1] = d1 * pos_traj_coeff[2];
	vel_traj_coeff[2] = d2 * pos_traj_coeff[3];
}

/*
 * input: 6th order polynomial's coefficients
 * output: 5th order polynomial's coefficients
 */
void differentiate_6th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff)
{
	const float d0 = 1.0f;
	const float d1 = 2.0f;
	const float d2 = 3.0f;
	const float d3 = 4.0f;
	const float d4 = 5.0f;
	const float d5 = 6.0f;
	vel_traj_coeff[0] = d0 * pos_traj_coeff[1];
	vel_traj_coeff[1] = d1 * pos_traj_coeff[2];
	vel_traj_coeff[2] = d2 * pos_traj_coeff[3];
	vel_traj_coeff[3] = d3 * pos_traj_coeff[4];
	vel_traj_coeff[4] = d4 * pos_traj_coeff[5];
	vel_traj_coeff[5] = d5 * pos_traj_coeff[6];
}

/*
 * input: 7th order polynomial's coefficients
 * output: 6th order polynomial's coefficients
 */
void differentiate_7th_polynomial(float *pos_traj_coeff, float *vel_traj_coeff)
{
	const float d0 = 1.0f;
	const float d1 = 2.0f;
	const float d2 = 3.0f;
	const float d3 = 4.0f;
	const float d4 = 5.0f;
	const float d5 = 6.0f;
	const float d6 = 7.0f;
	vel_traj_coeff[0] = d0 * pos_traj_coeff[1];
	vel_traj_coeff[1] = d1 * pos_traj_coeff[2];
	vel_traj_coeff[2] = d2 * pos_traj_coeff[3];
	vel_traj_coeff[3] = d3 * pos_traj_coeff[4];
	vel_traj_coeff[4] = d4 * pos_traj_coeff[5];
	vel_traj_coeff[5] = d5 * pos_traj_coeff[6];
	vel_traj_coeff[6] = d6 * pos_traj_coeff[7];
}
