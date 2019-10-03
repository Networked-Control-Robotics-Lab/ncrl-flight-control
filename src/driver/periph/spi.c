#include "stm32f4xx_conf.h"


void spi1_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	/* spi1 slave device:
	 * imu_cs: gpio_pin_a_4
	 * imu_sck: gpio_pin_a_5
	 * imu_miso: gpio_pin_a_6
	 * imu_mosi: gpio_pin_a_7
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL	
	};
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Chip select pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
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
		.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16,
		.SPI_FirstBit = SPI_FirstBit_MSB
	};
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}

uint8_t spi_read_write(SPI_TypeDef *spi_channel, uint8_t data)
{
	SPI_I2S_SendData(spi_channel, data);

	while(SPI_I2S_GetFlagStatus(spi_channel, SPI_FLAG_TXE) == RESET);

	while(SPI_I2S_GetFlagStatus(spi_channel, SPI_FLAG_RXNE) == RESET);

	return SPI_I2S_ReceiveData(spi_channel);
}
