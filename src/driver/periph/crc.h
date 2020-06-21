#ifndef __CRC_H__
#define __CRC_H__

void crc_init(void);
float calculate_crc_of_words(uint32_t *data_arr, int size);

#endif
