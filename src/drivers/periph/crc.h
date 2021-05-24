#ifndef __CRC_H__
#define __CRC_H__

void _crc_init(void);
uint32_t calculate_crc_of_words(uint32_t *data_arr, int size);

#endif
