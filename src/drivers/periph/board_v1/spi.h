#ifndef __SPI_H__
#define __SPI_H__

void spi1_init(void);
void spi3_init(void);

uint8_t spi_read_write(SPI_TypeDef *spi_channel, uint8_t data);
uint8_t spi3_read_write(uint8_t data);

#endif
