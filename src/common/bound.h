#ifndef __BOUND_H
#define __BOUND_H

#include <stdint.h>

void bound_float(float *val, float max, float min);
void bound_int16(int16_t *val, int16_t max, int16_t min);
void bound_uint16(uint16_t *val, uint16_t max, uint16_t min);
void bound_int32(int32_t *val, int32_t max, int32_t min);
void bound_uint32(uint32_t *val, uint32_t max, uint32_t min);

#endif
