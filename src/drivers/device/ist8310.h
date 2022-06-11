#ifndef __IST8310_H__
#define __IST8310_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define IST8310_ADDR 0xe

#define IST8310_REG_WIA   0x00
#define IST8310_REG_STAT1 0x02
#define IST8310_REG_DATA  0x03

#define IST8310_REG_CTRL1 0x0a
#define IST8310_REG_CTRL2 0x0b
#define IST8310_REG_AVG   0x41
#define IST8310_REG_PDCTL 0x42

#define IST8310_ODR_SINGLE 0x01
#define IST8310_ODR_10HZ   0x03
#define IST8310_ODR_20HZ   0x05
#define IST8310_ODR_50HZ   0x07
#define IST8310_ODR_100HZ  0x06

#define IST8310_CHIP_ID   0x10
#define IST8310_AVG_16    0x24
#define IST8310_PD_NORMAL 0xC0

#define IST8310_CNTRL2_RESET 0x01
#define IST8310_CNTRL2_DRPOL 0x04
#define IST8310_CNTRL2_DRENA 0x08

/* according to the datasheet, the scale factor is 0.3.
 * however, user manual v1.5 claims the scale factor is 3/20
 * (which is equal to 0.15).
 * the latter one seems to be correct according to the experiment.
 */
#define IST8310_RESOLUTION 0.15f

typedef struct {
	int16_t mag_unscaled[3];
	float mag_raw[3];
	float mag_lpf[3];

	float last_update_time;
	float update_rate;

	float bias_x;
	float bias_y;
	float bias_z;
	float rescale_x;
	float rescale_y;
	float rescale_z;
} ist8310_t;

void ist8130_init(void);
void ist8310_wait_until_stable(void);
bool ist8310_available(void);
void ist8310_read_sensor(void);

void ist8310_get_mag_raw(float *mag_raw);
void ist8310_get_mag_lpf(float *mag_lpf);
float ist8310_get_update_rate(void);
float ist8310_get_mag_raw_strength(void);
float ist8310_get_mag_lpf_strength(void);
void ist8310_undistortion(float *mag);

#endif
