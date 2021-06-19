#include <stddef.h>

float median_filter(float *measurements, size_t size)
{
	/* bubble sort */
	int i, j;
	for(i = 0; i < size; i++) {
		for(j = 0; j < size - 1 - i; j++) {
			if(measurements[j] < measurements[j + 1]) {
				float tmp;
				tmp = measurements[j];
				measurements[j] = measurements[j + 1];
				measurements[j + 1] = tmp;
			}
		}
	}

	/* pick the median value */
	return (measurements[size / 2] + measurements[size / 2 - 1]) / 2;
}
