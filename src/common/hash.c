#include <stddef.h>
#include "hash.h"

unsigned long hash_djb2(unsigned char *str)
{
	unsigned long hash = 5381;
	int c;

	while (str != NULL) {
		c = *str;
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
		str++;
	}

	return hash;
}
