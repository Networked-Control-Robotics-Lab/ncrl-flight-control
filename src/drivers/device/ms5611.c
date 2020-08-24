#include <stdint.h>
#include <math.h>
#include "delay.h"
#include "ms5611.h"
#include "debug_link.h"
#include "lpf.h"

uint16_t c1, c2, c3, c4, c5, c6;

float press_sea_level = 0.0f;
float press_now, temp_now;

float height_raw = 0.0f;
float height_filtered = 0.0f;

void ms5611_reset(void)
{
	ms5611_chip_select();
	spi_read_write(SPI3, 0x1e);
	ms5611_chip_deselect();

	blocked_delay_ms(100);
}

void ms5611_read_uint16(uint8_t address, uint16_t *data)
{
	uint8_t byte1, byte2;

	ms5611_chip_select();
	spi_read_write(SPI3, address);
	byte1 = spi_read_write(SPI3, 0x00);
	byte2 = spi_read_write(SPI3, 0x00);
	*data = ((uint16_t)byte1 << 8) | (uint16_t)byte2;
	ms5611_chip_deselect();
}

void ms5611_read_int24(uint8_t address, int32_t *data)
{
	uint8_t byte1, byte2, byte3;

	ms5611_chip_select();
	spi_read_write(SPI3, address);
	ms5611_chip_deselect();
	freertos_task_delay(1);

	ms5611_chip_select();
	spi_read_write(SPI3, 0x00);
	byte1 = spi_read_write(SPI3, 0x00);
	byte2 = spi_read_write(SPI3, 0x00);
	byte3 = spi_read_write(SPI3, 0x00);
	*data = ((int32_t)byte1 << 16) | ((int32_t)byte2 << 8) | (int32_t)byte3;
	ms5611_chip_deselect();
}

void ms5611_read_prom(void)
{
	ms5611_chip_select();
	ms5611_read_uint16(0xa2, &c1);
	ms5611_read_uint16(0xa4, &c2);
	ms5611_read_uint16(0xa6, &c3);
	ms5611_read_uint16(0xa8, &c4);
	ms5611_read_uint16(0xaa, &c5);
	ms5611_read_uint16(0xac, &c6);
	ms5611_chip_deselect();
}

void ms5611_init(void)
{
	ms5611_reset();
	ms5611_read_prom();

	ms5611_read_pressure();
	ms5611_set_sea_level();
}

void ms5611_read_pressure(void)
{
	int32_t d1, d2;
	int64_t off, sens, dt;

	ms5611_read_int24(0x40, &d1);
	ms5611_read_int24(0x50, &d2);

	dt = (int64_t)d2 - (int64_t)c5 * (1 << 8);
	int32_t temp32 = 2000 + (dt * (int64_t)c6) / (1 << 23);

	/* second order temperature compensation (<20 degree c) */
	int64_t t2 = 0, sens2 = 0, off2 = 0;
	if(temp32 < 2000) {
		t2 = (dt * dt) / (1 << 31);
		int64_t temp_2000_sqrt = temp32 - 2000;
		off2 = 2.5f * (temp_2000_sqrt * temp_2000_sqrt);
		sens2 =  1.25f * (temp_2000_sqrt * temp_2000_sqrt);

		temp32 -= t2;
		off -= off2;
		sens -= sens2;
	}

	off = (int64_t)c2 * (1 << 16) + ((int32_t)c4 * dt) / (1 << 7);
	sens = (int64_t)c1 * (1 << 15) + ((int32_t)c3 * dt) / (1 << 8);
	int32_t pressure32 = ((d1 * sens) / (1 << 21) - off) / (1 << 15);

	temp_now = (float)temp32 * 0.01f; //[deg c]
	press_now = (float)pressure32 * 0.01f; //[mbar]
}

float ms5611_get_relative_height(void)
{
	height_raw = 44330.0f * (1.0f - pow(press_now / press_sea_level, 0.1902949f));
	lpf(height_raw, &height_filtered, 0.05f);

	return height_filtered;
}

void ms5611_set_sea_level(void)
{
	press_sea_level = press_now;
}

void send_barometer_debug_message(debug_msg_t *payload)
{
	float press_now_bar = press_now * 0.001f;

	pack_debug_debug_message_header(payload, MESSAGE_ID_BAROMETER);
	pack_debug_debug_message_float(&press_now_bar, payload);
	pack_debug_debug_message_float(&temp_now, payload);
	pack_debug_debug_message_float(&height_filtered, payload);
}
