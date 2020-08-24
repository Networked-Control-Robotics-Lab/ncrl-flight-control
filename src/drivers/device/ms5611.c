#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "ms5611.h"
#include "debug_link.h"
#include "lpf.h"

SemaphoreHandle_t ms5611_task_semphr;

ms5611_t ms5611;

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
	ms5611_read_uint16(0xa2, &ms5611.c1);
	ms5611_read_uint16(0xa4, &ms5611.c2);
	ms5611_read_uint16(0xa6, &ms5611.c3);
	ms5611_read_uint16(0xa8, &ms5611.c4);
	ms5611_read_uint16(0xaa, &ms5611.c5);
	ms5611_read_uint16(0xac, &ms5611.c6);
	ms5611_chip_deselect();
}

void ms5611_init(void)
{
	ms5611_task_semphr = xSemaphoreCreateBinary();

	ms5611_reset();
	ms5611_read_prom();
}

void ms5611_read_pressure(void)
{
	int32_t d1, d2;
	int64_t off, sens, dt;

	ms5611_read_int24(0x40, &d1);
	ms5611_read_int24(0x50, &d2);

	dt = (int64_t)d2 - (int64_t)ms5611.c5 * (1 << 8);
	int32_t temp32 = 2000 + (dt * (int64_t)ms5611.c6) / (1 << 23);

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

	off = (int64_t)ms5611.c2 * (1 << 16) + ((int32_t)ms5611.c4 * dt) / (1 << 7);
	sens = (int64_t)ms5611.c1 * (1 << 15) + ((int32_t)ms5611.c3 * dt) / (1 << 8);
	int32_t pressure32 = ((d1 * sens) / (1 << 21) - off) / (1 << 15);

	ms5611.temp_raw = (float)temp32 * 0.01f; //[deg c]
	ms5611.press_raw = (float)pressure32 * 0.01f; //[mbar]

	lpf(ms5611.press_raw, &ms5611.press_lpf, 0.045f);
}

static void ms5611_calc_relative_altitude_and_velocity(void)
{
	/* calculate relative height */
	ms5611.rel_alt = 44330.0f * (1.0f - pow(ms5611.press_lpf / ms5611.press_sea_level, 0.1902949f));

	static int diff_prescaler = 4;

	/* down-sampled numerical differentiation (relative velocity) */
	diff_prescaler--;
	if(diff_prescaler == 0) {
		ms5611.rel_vel_raw = (ms5611.rel_alt - ms5611.rel_alt_last) * 100; //FIXME
		ms5611.rel_alt_last = ms5611.rel_alt;
		diff_prescaler = 4;

		/* low pass filtering */
		lpf(ms5611.rel_vel_raw, &ms5611.rel_vel_lpf, 0.1f);
	}
}

void ms5611_set_sea_level(void)
{
	ms5611.press_sea_level = ms5611.press_lpf;
}

float ms5611_get_pressure(void)
{
	return ms5611.press_lpf;
}

float ms5611_get_relative_altitude(void)
{
	return ms5611.rel_alt;
}

float ms5611_get_relative_altitude_rate(void)
{
	return ms5611.rel_vel_lpf;
}

void send_barometer_debug_message(debug_msg_t *payload)
{
	float press_lpf_bar = ms5611.press_lpf * 0.001;

	pack_debug_debug_message_header(payload, MESSAGE_ID_BAROMETER);
	pack_debug_debug_message_float(&press_lpf_bar, payload);
	pack_debug_debug_message_float(&ms5611.temp_raw, payload);
	pack_debug_debug_message_float(&ms5611.rel_alt, payload);
	pack_debug_debug_message_float(&ms5611.rel_vel_lpf, payload);
}

void ms5611_driver_semaphore_handler(void)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(ms5611_task_semphr, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void ms5611_driver_task(void *param)
{
	/* initialize sea level for estimating relative altitude*/
	ms5611_read_pressure();
	ms5611_set_sea_level();

	TickType_t sleep_time = OS_TICK / 4000; //1/4000s
	while(1) {
		while(xSemaphoreTake(ms5611_task_semphr, sleep_time) == pdFALSE);

		ms5611_read_pressure();
		ms5611_calc_relative_altitude_and_velocity();
	}
}
