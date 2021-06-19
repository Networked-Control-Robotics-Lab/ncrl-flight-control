#include "delay.h"
#include "lidar_lite.h"
#include "sw_i2c.h"

#if 0
static void lidar_read_half_word(uint8_t addr, uint16_t *data)
{
}
#endif

void lidar_blocked_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_blocked_start();
	sw_i2c_blocked_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(addr);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_send_byte(data);
	sw_i2c_blocked_wait_ack();
	sw_i2c_blocked_stop();
}

void lidar_write_byte(uint8_t addr, uint8_t data)
{
	sw_i2c_start();
	sw_i2c_send_byte((LIDAR_DEV_ADDRESS << 1) | 0);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(addr);
	sw_i2c_wait_ack();
	sw_i2c_send_byte(data);
	sw_i2c_wait_ack();
	sw_i2c_stop();
}

void lidar_lite_init(void)
{
	/* reset lidar */
	lidar_blocked_write_byte(LIDAR_ACQ_COMMAND_REG, 0x00);
	blocked_delay_ms(1000);

	/* continuous reading mode */
	lidar_blocked_write_byte(LIDAR_MEASURE_COUNT_REG, 0xff);
	blocked_delay_ms(10);

	/* measurement rate */
	lidar_blocked_write_byte(LIDAR_MEASURE_DELAY_REG, 0x02);
	blocked_delay_ms(10);

	/* fast reading mode */
	lidar_blocked_write_byte(LIDAR_ACQ_CONFIG_REG, 0x21);
	blocked_delay_ms(10);

	/* start distance measurement */
	lidar_blocked_write_byte(LIDAR_ACQ_COMMAND_REG, 0x04);
	blocked_delay_ms(10);
}

void lidar_lite_isr_handler(void)
{
}
