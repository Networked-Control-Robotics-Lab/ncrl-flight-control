#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "ms5611.h"
#include "debug_link.h"
#include "lpf.h"

#define MBAR_TO_PASCAL(press) (press * 100.0f)

SemaphoreHandle_t ms5611_task_semphr;

ms5611_t ms5611;

bool ms5611_available(void)
{
	return true; //TODO: data lost checking?
}

void ms5611_reset(void)
{
	ms5611_chip_select();
	spi_read_write(SPI3, 0x1e);
	ms5611_chip_deselect();

	blocked_delay_ms(100);
}

void ms5611_read_uint16(uint8_t address, uint32_t *data)
{
	uint8_t byte1, byte2;

	ms5611_chip_select();
	spi_read_write(SPI3, address);
	byte1 = spi_read_write(SPI3, 0x00);
	byte2 = spi_read_write(SPI3, 0x00);
	*data = ((uint32_t)byte1 << 8) | (uint32_t)byte2;
	ms5611_chip_deselect();
}

void ms5611_read_int24(uint8_t address, int32_t *data)
{
	uint8_t byte1, byte2, byte3;

	ms5611_chip_select();
	spi3_read_write(address);
	ms5611_chip_deselect();
	freertos_task_delay(20);

	ms5611_chip_select();
	spi3_read_write(0x00);
	byte1 = spi3_read_write(0x00);
	byte2 = spi3_read_write(0x00);
	byte3 = spi3_read_write(0x00);
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

void ms5611_wait_until_stable(void)
{
	/* wait until barometer is stable */
	while(ms5611.press_lpf < 800 || ms5611.press_lpf > 1200) {
		freertos_task_delay(1);
	}

	ms5611.init_finished = true;
	ms5611.press_sea_level = ms5611.press_lpf;
}

void ms5611_read_pressure(void)
{
	int32_t d1, d2;
	int64_t off, sens, dt;

	ms5611_read_int24(MS5611_D1_CONVERT_OSR4096, &d1);
	ms5611_read_int24(MS5611_D2_CONVERT_OSR4096, &d2);

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

	lpf_first_order(ms5611.press_raw, &ms5611.press_lpf, 0.18f);
}

static void ms5611_calc_relative_altitude_and_velocity(void)
{
	//wait until pressure data is stable
	if(ms5611.init_finished == false) {
		return;
	}

	//pressure: [pascal] = [N/m^2] = [kg/(m*s^2)]
	//air density: 1.2754[kg/m^3]
	//gravitational acceleration: 9.81[m/s^2]
	//--------------------------------------------------
	//linear equation for calculating altitude (valid under ~500m)
	//height = pressure_diff / (air_density * gravity)
	float press_diff = ms5611.press_sea_level - ms5611.press_lpf;
	ms5611.rel_alt = MBAR_TO_PASCAL(press_diff) * 0.07992535611;

	//ms5611.rel_alt = 44330.0f * (1.0f - pow(ms5611.press_raw / ms5611.press_sea_level, 0.1902949f));

	if(ms5611.velocity_ready == false) {
		ms5611.velocity_ready = true;
		ms5611.rel_vel_lpf = 0.0f;
		ms5611.rel_vel_raw = 0.0f;
		ms5611.rel_alt_last = 0.0f;
	} else {
		/* low pass filtering */
		ms5611.rel_vel_raw = (ms5611.rel_alt - ms5611.rel_alt_last) * 50;
		ms5611.rel_alt_last = ms5611.rel_alt;
		lpf_first_order(ms5611.rel_vel_raw, &ms5611.rel_vel_lpf, 0.35);
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

void ms5611_driver_semaphore_handler(BaseType_t *higher_priority_task_woken)
{
	xSemaphoreGiveFromISR(ms5611_task_semphr, higher_priority_task_woken);
}

void ms5611_driver_task(void *param)
{
	while(1) {
		while(xSemaphoreTake(ms5611_task_semphr, portMAX_DELAY) == pdFALSE);

		ms5611_read_pressure();
		ms5611_calc_relative_altitude_and_velocity();
	}
}

void ms5611_register_task(const char *task_name, configSTACK_DEPTH_TYPE stack_size,
                          UBaseType_t priority)
{
	xTaskCreate(ms5611_driver_task, task_name, stack_size, NULL, priority, NULL);
}
