unsigned long hash_djb2(unsigned char *str)
{
	unsigned long hash = 5381;
	int c;

	while ((c = *str++) != 0) {
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}

	return hash;
}
