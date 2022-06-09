#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sw_i2c.h"
#include "delay.h"
#include "ist8310.h"
#include "sys_time.h"
#include "gpio.h"
#include "lpf.h"
#include "ins_sensor_sync.h"

ist8310_t ist8310 = {
	.bias_x = 0.0f,
	.bias_y = 0.0f,
	.bias_z = 0.0f,
	.squared_semi_axis_size_x = 1.0f,
	.squared_semi_axis_size_y = 1.0f,
	.squared_semi_axis_size_z = 1.0f,
	.div_squared_semi_axis_size_x = 1.0f,
	.div_squared_semi_axis_size_y = 1.0f,
	.div_squared_semi_axis_size_z = 1.0f
};

float ist8310_lpf_gain;

bool ist8310_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - ist8310.last_update_time) > 300) {
		return false;
	}
	return true;
}

uint8_t ist8310_read_byte(uint8_t addr)
{
	uint8_t data;

	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_wait_ack();
	data = sw_i2c_read_byte();
	sw_i2c_nack();
	sw_i2c_stop();

	return data;
}

void ist8310_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(data);
	sw_i2c_wait_ack();
	sw_i2c_stop();
}

uint8_t ist8310_blocked_read_byte(uint8_t addr)
{
	uint8_t data;

	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_blocked_wait_ack();
	data = sw_i2c_blocked_read_byte();
	sw_i2c_blocked_nack();
	sw_i2c_blocked_stop();

	return data;
}

void ist8310_blocked_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(data);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_stop();
}

void ist8310_read_bytes(uint8_t addr, uint8_t *data, int size)
{
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_start();
	sw_i2c_send_byte((IST8310_ADDR << 1) | 1);
	sw_i2c_wait_ack();
	for(int i = 0; i < size; i++) {
		data[i] = sw_i2c_read_byte();
		if(i == (size-1)) {
			/* send nack if all bytes are received */
			sw_i2c_nack();
		} else {
			/* send ack for requesting next byte */
			sw_i2c_ack();
		}
	}
	sw_i2c_stop();
}

static uint8_t ist8310_read_who_i_am(void)
{
	uint8_t id = ist8310_blocked_read_byte(IST8310_REG_WIA);
	blocked_delay_ms(5);
	return id;
}

void ist8310_reset(void)
{
	ist8310_blocked_write_byte(IST8310_REG_CTRL2, 0x01);
	blocked_delay_ms(100);
}

void ist8130_init(void)
{
	while(ist8310_read_who_i_am() != IST8310_CHIP_ID);

	ist8310_reset();

	ist8310_blocked_write_byte(IST8310_REG_AVG, IST8310_AVG_16);
	blocked_delay_ms(100);

	ist8310_blocked_write_byte(IST8310_REG_PDCTL, IST8310_PD_NORMAL);
	blocked_delay_ms(100);

	ist8310.last_update_time = get_sys_time_s();

	//sampling time = 0.02s (50Hz), cutoff frequency = 20Hz
	lpf_first_order_init(&ist8310_lpf_gain, 0.02, 5);
}

void ist8310_wait_until_stable(void)
{
	while(ins_compass_sync_buffer_available() != true) {
		freertos_task_delay(2.5);
	}
}

void ist8310_cancel_bias(float *mag)
{
	mag[0] -= ist8310.bias_x;
	mag[1] -= ist8310.bias_y;
	mag[2] -= ist8310.bias_z;
}

void ist8310_undistortion(float *mag)
{
	/* standard equation of ellipsoid:
	 * (x - x0)^2/A + (y - y0)^2/B + (z - z0)^2/C = 1 */

	/* resume distorted magnatic field from ellipsoid to sphere.
	 * this operation do not preserve the actual size of the data,
	 * and requires normalization before calculating the heading
	 * angle */
	mag[0] *= ist8310.div_squared_semi_axis_size_x;
	mag[1] *= ist8310.div_squared_semi_axis_size_y;
	mag[2] *= ist8310.div_squared_semi_axis_size_z;
}

void ist8310_read_sensor(void)
{
	/* check "IST8310 User Manual v1.5" for details */

	//sigle measurement mode
	ist8310_write_byte(IST8310_REG_CTRL1, IST8310_ODR_SINGLE);

	//wait 6ms for 16x average
	TickType_t last_wake_time = xTaskGetTickCount();
	vTaskDelayUntil(&last_wake_time, CONVERT_MS_TO_TICK(6));

	/* read sensor datas */
	uint8_t buf[6];
	ist8310_read_bytes(IST8310_REG_DATA, buf, 6);

	/* composite unscaled data */
	ist8310.mag_unscaled[0] = (((int16_t)buf[3]) << 8 | buf[2]);
	ist8310.mag_unscaled[1] = (((int16_t)buf[1]) << 8 | buf[0]);
	ist8310.mag_unscaled[2] = (((int16_t)buf[5]) << 8 | buf[4]);

	/* convert unscaled data to raw data (NED frame) */
	ist8310.mag_raw[0] = ist8310.mag_unscaled[0] * IST8310_RESOLUTION * 0.01;
	ist8310.mag_raw[1] = ist8310.mag_unscaled[1] * IST8310_RESOLUTION * 0.01;
	ist8310.mag_raw[2] = ist8310.mag_unscaled[2] * IST8310_RESOLUTION * 0.01;

	/* low pass filtering */
	lpf_first_order(ist8310.mag_raw[0], &(ist8310.mag_lpf[0]), ist8310_lpf_gain);
	lpf_first_order(ist8310.mag_raw[1], &(ist8310.mag_lpf[1]), ist8310_lpf_gain);
	lpf_first_order(ist8310.mag_raw[2], &(ist8310.mag_lpf[2]), ist8310_lpf_gain);

	/* calculate update frequency */
	float curr_time = get_sys_time_s();
	float elapsed_time = curr_time - ist8310.last_update_time;
	ist8310.update_rate = 1.0f / elapsed_time;
	ist8310.last_update_time = curr_time;

	/* update timer only if data is valid */
	if(ist8310.mag_raw[0] != 0 || ist8310.mag_raw[1] != 0 || ist8310.mag_raw[2] != 0) {
		ins_compass_sync_buffer_push(ist8310.mag_lpf);
		ist8310.last_update_time = get_sys_time_ms();
	}
}

void ist8310_get_mag_raw(float *mag_raw)
{
	mag_raw[0] = ist8310.mag_raw[0];
	mag_raw[1] = ist8310.mag_raw[1];
	mag_raw[2] = ist8310.mag_raw[2];

	/* apply calibration here since the function will called by the flight control task
	 * (which has the highest priority) */
	ist8310_cancel_bias(mag_raw);
}

void ist8310_get_mag_lpf(float *mag_lpf)
{
	mag_lpf[0] = ist8310.mag_lpf[0];
	mag_lpf[1] = ist8310.mag_lpf[1];
	mag_lpf[2] = ist8310.mag_lpf[2];

	/* apply calibration here since the function will called by the flight control task
	 * (which has the highest priority) */
	ist8310_cancel_bias(mag_lpf);
}

float ist8310_get_mag_raw_strength(void)
{
	float mx = ist8310.mag_raw[0];
	float my = ist8310.mag_raw[1];
	float mz = ist8310.mag_raw[2];

	float mag_strength;
	arm_sqrt_f32(mx*mx + my*my + mz*mz, &mag_strength);

	return mag_strength;
}

float ist8310_get_mag_lpf_strength(void)
{
	float mx = ist8310.mag_lpf[0];
	float my = ist8310.mag_lpf[1];
	float mz = ist8310.mag_lpf[2];

	float mag_strength;
	arm_sqrt_f32(mx*mx + my*my + mz*mz, &mag_strength);

	return mag_strength;
}

float ist8310_get_update_rate(void)
{
	return ist8310.update_rate;
}
