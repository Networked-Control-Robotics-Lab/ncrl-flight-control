#ifndef __IST8310_H__
#define __IST8310_H__

#include <stdint.h>

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

#define IST8310_RESOLUTION_3MG 3.0f

typedef struct {
	int16_t mag_unscaled[3];
	float mag_raw[3];
} ist8310_t;

void ist8130_init(void);
void ist8310_get_raw_mag(float *mag_raw);

void ist8310_task_handler(void);

#endif
