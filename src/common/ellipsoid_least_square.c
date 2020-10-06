#include "matrix.h"

MAT_ALLOC(LS_A, 6, 6);
MAT_ALLOC(LS_A_INV, 6, 6);
MAT_ALLOC(LS_b, 1, 6);
MAT_ALLOC(LS_x, 1, 6);

float x_bar;
float y_bar;
float z_bar;
float xy_bar;
float xz_bar;
float yz_bar;
float xx_bar;
float yy_bar;
float zz_bar;
float xxy_bar;
float xxz_bar;
float xyy_bar;
float yyz_bar;
float xzz_bar;
float yzz_bar;
float xxx_bar;
float yyy_bar;
float zzz_bar;
float yyyy_bar;
float zzzz_bar;
float xxyy_bar;
float xxzz_bar;
float yyzz_bar;

void ellipsoid_least_square_fitting_init(void)
{
	MAT_INIT(LS_A, 6, 6);
	MAT_INIT(LS_A_INV, 6, 6);
	MAT_INIT(LS_b, 1, 6);
	MAT_INIT(LS_x, 1, 6);

	x_bar = 0.0f;
	y_bar = 0.0f;
	z_bar = 0.0f;
	xy_bar = 0.0f;
	xz_bar = 0.0f;
	yz_bar = 0.0f;
	xx_bar = 0.0f;
	yy_bar = 0.0f;
	zz_bar = 0.0f;
	xxy_bar = 0.0f;
	xxz_bar = 0.0f;
	xyy_bar = 0.0f;
	yyz_bar = 0.0f;
	xzz_bar = 0.0f;
	yzz_bar = 0.0f;
	xxx_bar = 0.0f;
	yyy_bar = 0.0f;
	zzz_bar = 0.0f;
	yyyy_bar = 0.0f;
	zzzz_bar = 0.0f;
	xxyy_bar = 0.0f;
	xxzz_bar = 0.0f;
	yyzz_bar = 0.0f;
}

void ellipsoid_least_square_fitting(float *x0, float *y0, float *z0, float *A, float *B, float *C)
{
	mat_data(LS_A)[0*6 + 0] = yyyy_bar;
	mat_data(LS_A)[0*6 + 1] = yyzz_bar;
	mat_data(LS_A)[0*6 + 2] = xyy_bar;
	mat_data(LS_A)[0*6 + 3] = yyy_bar;
	mat_data(LS_A)[0*6 + 4] = yyz_bar;
	mat_data(LS_A)[0*6 + 5] = yy_bar;

	mat_data(LS_A)[1*6 + 0] = yyzz_bar;
	mat_data(LS_A)[1*6 + 1] = zzzz_bar;
	mat_data(LS_A)[1*6 + 2] = xzz_bar;
	mat_data(LS_A)[1*6 + 3] = yzz_bar;
	mat_data(LS_A)[1*6 + 4] = zzz_bar;
	mat_data(LS_A)[1*6 + 5] = zz_bar;

	mat_data(LS_A)[2*6 + 0] = xyy_bar;
	mat_data(LS_A)[2*6 + 1] = xzz_bar;
	mat_data(LS_A)[2*6 + 2] = xx_bar;
	mat_data(LS_A)[2*6 + 3] = xy_bar;
	mat_data(LS_A)[2*6 + 4] = xz_bar;
	mat_data(LS_A)[2*6 + 5] = x_bar;

	mat_data(LS_A)[3*6 + 0] = yyy_bar;
	mat_data(LS_A)[3*6 + 1] = yzz_bar;
	mat_data(LS_A)[3*6 + 2] = xy_bar;
	mat_data(LS_A)[3*6 + 3] = yy_bar;
	mat_data(LS_A)[3*6 + 4] = yz_bar;
	mat_data(LS_A)[3*6 + 5] = y_bar;

	mat_data(LS_A)[4*6 + 0] = yyz_bar;
	mat_data(LS_A)[4*6 + 1] = zzz_bar;
	mat_data(LS_A)[4*6 + 2] = xz_bar;
	mat_data(LS_A)[4*6 + 3] = yz_bar;
	mat_data(LS_A)[4*6 + 4] = zz_bar;
	mat_data(LS_A)[4*6 + 5] = z_bar;

	mat_data(LS_A)[5*6 + 0] = yy_bar;
	mat_data(LS_A)[5*6 + 1] = zz_bar;
	mat_data(LS_A)[5*6 + 2] = x_bar;
	mat_data(LS_A)[5*6 + 3] = y_bar;
	mat_data(LS_A)[5*6 + 4] = z_bar;
	mat_data(LS_A)[5*6 + 5] = 1.0f;

	mat_data(LS_b)[0] = -xxyy_bar;
	mat_data(LS_b)[1] = -xxzz_bar;
	mat_data(LS_b)[2] = -xxx_bar;
	mat_data(LS_b)[3] = -xxy_bar;
	mat_data(LS_b)[4] = -xxz_bar;
	mat_data(LS_b)[5] = -xx_bar;

	MAT_INV(&LS_A, &LS_A_INV); //XXX: check inversion is valid or not
	MAT_MULT(&LS_A_INV, &LS_b, &LS_x);

	*x0 = -0.5 * mat_data(LS_x)[2]; //XXX: use fast squared root function
	*y0 = -mat_data(LS_x)[3] / (2.0f * mat_data(LS_x)[0]);
	*z0 = mat_data(LS_x)[4] / (2.0f * mat_data(LS_x)[1]);
	*A = sqrt(mat_data(LS_x)[0]*mat_data(LS_x)[0] + mat_data(LS_x)[1]*mat_data(LS_x)[1] + mat_data(LS_x)[2]*mat_data(LS_x)[2]);
	*B = *A / sqrt(mat_data(LS_x)[0]);
	*C = *A / sqrt(mat_data(LS_x)[1]);
}
