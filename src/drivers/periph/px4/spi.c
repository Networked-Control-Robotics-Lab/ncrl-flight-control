#include <stdbool.h>
#include "stm32f4xx_conf.h"
#include "proj_config.h"

//TODO: porting of ms5611 is not complete

uint8_t spi3_tx_buf;
uint8_t spi3_rx_buf;

/* <spi1>
 * usage: mpu6000 (imu)
 * cs: gpio_pin_c_2
 * sck: gpio_pin_a_5
 * miso: gpio_pin_a_6
 * mosi: gpio_pin_a_7
 */
void spi1_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* chip selection pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	SPI_InitTypeDef SPI_InitStruct = {
		.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode = SPI_Mode_Master,
		.SPI_DataSize = SPI_DataSize_8b,
		.SPI_CPOL = SPI_CPOL_High,
		.SPI_CPHA = SPI_CPHA_2Edge,
		.SPI_NSS = SPI_NSS_Soft,
		.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4,
		.SPI_FirstBit = SPI_FirstBit_MSB,
		.SPI_CRCPolynomial = 7
	};
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}

/* <spi3>
 * usage: ms5611 (barometer)
 * cs: gpio_pin_a_15
 * sck: gpio_pin_b_3
 * miso: gpio_pin_b_4
 * mosi: gpio_pin_b_5
 */
void spi3_init(void)
{
#if 0
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Chip select pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	SPI_InitTypeDef SPI_InitStruct = {
		.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode = SPI_Mode_Master,
		.SPI_DataSize = SPI_DataSize_8b,
		.SPI_CPOL = SPI_CPOL_High,
		.SPI_CPHA = SPI_CPHA_2Edge,
		.SPI_NSS = SPI_NSS_Soft,
		.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4,
		.SPI_FirstBit = SPI_FirstBit_MSB,
		.SPI_CRCPolynomial = 7
	};
	SPI_Init(SPI3, &SPI_InitStruct);

	SPI_Cmd(SPI3, ENABLE);
#endif
}

uint8_t spi_read_write(SPI_TypeDef *spi_channel, uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(spi_channel, SPI_FLAG_TXE) == RESET);
	SPI_I2S_SendData(spi_channel, data);

	while(SPI_I2S_GetFlagStatus(spi_channel, SPI_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(spi_channel);
}

uint8_t spi1_read_write(uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data);

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI1);
}

uint8_t spi3_read_write(uint8_t data)
{
#if 0
	return spi_read_write(SPI3, data);
#else
	return 0;
#endif
}

void spi1_chip_select(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_2);
}

void spi1_chip_deselect(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_2);
}

void spi3_chip_select(void)
{
#if 0
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
#endif
}

void spi3_chip_deselect(void)
{
#if 0
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
#endif
}
