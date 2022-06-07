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
#include "ins_sensor_sync.h"
#include "coroutine.h"
#include "sys_time.h"
#include "barometer.h"
#include "ins_sensor_sync.h"
#include "board_porting.h"
#include "proj_config.h"

#define POW2(x) ((x) * (x))
#define MBAR_TO_PASCAL(press) (press * 100.0f)

#define MS5611_UPDATE_FREQ (50)

ms5611_t ms5611;

bool ms5611_available(void)
{
	return true; //TODO: data lost checking?
}

void ms5611_reset(void)
{
	barometer_spi_chip_select();
	barometer_spi_read_write(0x1e);
	barometer_spi_chip_deselect();

	blocked_delay_ms(100);
}

void ms5611_read_uint16(uint8_t address, uint32_t *data)
{
	uint8_t byte1, byte2;

	barometer_spi_chip_select();
	barometer_spi_read_write(address);
	byte1 = barometer_spi_read_write(0x00);
	byte2 = barometer_spi_read_write(0x00);
	*data = ((uint32_t)byte1 << 8) | (uint32_t)byte2;
	barometer_spi_chip_deselect();
}

void ms5611_read_int24_addr(uint8_t address)
{
	barometer_spi_chip_select();
	barometer_spi_read_write(address);
	barometer_spi_chip_deselect();
}

void ms5611_read_int24_data(int32_t *data)
{
	uint8_t byte1, byte2, byte3;

	barometer_spi_chip_select();
	barometer_spi_read_write(0x00);
	byte1 = barometer_spi_read_write(0x00);
	byte2 = barometer_spi_read_write(0x00);
	byte3 = barometer_spi_read_write(0x00);
	*data = ((int32_t)byte1 << 16) | ((int32_t)byte2 << 8) | (int32_t)byte3;
	barometer_spi_chip_deselect();
}

void ms5611_read_prom(void)
{
	barometer_spi_chip_select();
	ms5611_read_uint16(0xa2, &ms5611.c1);
	ms5611_read_uint16(0xa4, &ms5611.c2);
	ms5611_read_uint16(0xa6, &ms5611.c3);
	ms5611_read_uint16(0xa8, &ms5611.c4);
	ms5611_read_uint16(0xaa, &ms5611.c5);
	ms5611_read_uint16(0xac, &ms5611.c6);
	barometer_spi_chip_deselect();
}

void ms5611_init(void)
{
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

float ms5611_get_update_freq(void)
{
	return ms5611.update_freq;
}

void ms5611_convert_pressure_temperature(int32_t d1, int32_t d2)
{
	int64_t off, sens, dt;

	dt = (int32_t)d2 - ((int32_t)ms5611.c5 << 8);
	int32_t temp = 2000 + (int32_t)(((int64_t)dt * ms5611.c6) >> 23);

	off = ((int64_t)ms5611.c2 << 16) + (((int64_t)ms5611.c4 * dt) >> 7);
	sens = ((int64_t)ms5611.c1 << 15) + (((int64_t)ms5611.c3 * dt) >> 8);

	/* second order temperature compensation (<20 degree c) */
	if(temp < 2000) {
		int32_t t2 = POW2(dt) >> 31;

		int64_t f = POW2((int64_t)temp - 2000);
		int64_t off2 = 5 * f >> 1;
		int64_t sens2 = 5 * f >> 2;

		/* second order temperature compensation (<-15 degree c) */
		if (temp < -1500) {
			int64_t f2 = POW2(temp + 1500);
			off2 += 7 * f2;
			sens2 += 11 * f2 >> 1;
		}

		temp -= t2;
		off  -= off2;
		sens -= sens2;
	}

	int64_t pressure = (((d1 * sens) >> 21) - off) >> 15;

	ms5611.temp_raw = (float)temp / 100.0f;      //[deg c]
	ms5611.press_raw = (float)pressure / 100.0f; //[mbar]

	lpf_first_order(ms5611.press_raw, &ms5611.press_lpf, 0.1f); //0.18f
}

static void ms5611_calc_relative_altitude_and_velocity(BaseType_t *higher_priority_task_woken)
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
		ms5611.rel_vel_raw = (ms5611.rel_alt - ms5611.rel_alt_last) * MS5611_UPDATE_FREQ;
		ms5611.rel_alt_last = ms5611.rel_alt;
		lpf_first_order(ms5611.rel_vel_raw, &ms5611.rel_vel_lpf, 0.35);

		ins_barometer_sync_buffer_push_from_isr(ms5611.rel_alt, ms5611.rel_vel_lpf,
		                                        higher_priority_task_woken);
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
	float barometer_update_freq = barometer_get_update_freq();
	float press_lpf_bar = ms5611.press_lpf * 0.001;

	pack_debug_debug_message_header(payload, MESSAGE_ID_BAROMETER);
	pack_debug_debug_message_float(&barometer_update_freq, payload);
	pack_debug_debug_message_float(&press_lpf_bar, payload);
	pack_debug_debug_message_float(&ms5611.temp_raw, payload);
	pack_debug_debug_message_float(&ms5611.rel_alt, payload);
	pack_debug_debug_message_float(&ms5611.rel_vel_lpf, payload);
}

void ms5611_driver_handler(BaseType_t *higher_priority_task_woken)
{
	static bool do_initial_start = true;
	if(do_initial_start == true) {
		do_initial_start = false;

		/* trigger ms5611 d1 conversion, need to wait 10ms for
		 * getting the result */
		ms5611_read_int24_addr(MS5611_D1_CONVERT_OSR4096);
		return;
	}

	CR_START();

	/* get the result of d1 conversion */
	ms5611_read_int24_data(&ms5611.d1);

	CR_YIELD();

	/* trigger ms5611 d2 conversion, need to wait 10ms for getting
	 * the result */
	ms5611_read_int24_addr(MS5611_D2_CONVERT_OSR4096);

	CR_YIELD();

	/* get the result of d2 conversion */
	ms5611_read_int24_data(&ms5611.d2);

	/* convert ms5611 register to pressure and temperature value */
	ms5611_convert_pressure_temperature(ms5611.d1, ms5611.d2);

	/* convert pressure and temperature to height and velocity value */
	ms5611_calc_relative_altitude_and_velocity(higher_priority_task_woken);

	float curr_time = get_sys_time_s();
	ms5611.update_freq = 1.0f / (curr_time - ms5611.last_read_time);
	ms5611.last_read_time = curr_time;

	/* trigger ms5611 d1 conversion, need to wait 10ms for getting
	 * the result */
	ms5611_read_int24_addr(MS5611_D1_CONVERT_OSR4096);

	CR_END();
}

#define BAROMETER_PRESCALER_RELOAD 4 //100Hz
void ms5611_driver_trigger_handler(void)
{
#if (ENABLE_BAROMETER != 0)
	static int barometer_cnt = BAROMETER_PRESCALER_RELOAD;

	BaseType_t higher_priority_task_woken = pdFALSE;

	/* barometer */
	barometer_cnt--;
	if(barometer_cnt == 0) {
		barometer_cnt = BAROMETER_PRESCALER_RELOAD;
		ms5611_driver_handler(&higher_priority_task_woken);
	}
#endif
}
