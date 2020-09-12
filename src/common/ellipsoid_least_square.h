#ifndef __ELLIPSOID_LEAST_SQUARE_H__
#define __ELLIPSOID_LEAST_SQUARE_H__

void ellipsoid_least_square_fitting_init(void);
void ellipsoid_least_square_fitting(float *x0, float *y0, float *z0, float *A, float *B, float *C);

#endif
