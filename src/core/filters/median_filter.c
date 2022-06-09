#include <stddef.h>

float median_filter(float *measurements, size_t size)
{
	/* bubble sort */
	int i, j;
	for (i = 0; i < size - 1; i++) {
		for (j = 0; j < size - i - 1; j++) {
			if (measurements[j] > measurements[j+1]) {
				float tmp = measurements[j];
				measurements[j] = measurements[j+1];
				measurements[j+1] = tmp;
			}
		}
	}

	/* pick the median value */
	return measurements[size / 2];
}
