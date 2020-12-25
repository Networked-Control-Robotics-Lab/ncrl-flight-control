#include <stdbool.h>

#include "arm_math.h"
#include "matrix.h"

volatile arm_status mat_op_status = 0;

void matrix_reset(float *data, int row_num, int column_num)
{
	int r, c;
	for(r = 0; r < row_num; r++) {
		for(c = 0; c < column_num; c++) {
			data[r * column_num + c] = 0;
		}
	}
}
