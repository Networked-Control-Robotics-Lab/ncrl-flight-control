#ifndef __HMC5983_H
#define __HMC5983_H

#include "vector.h"

#define HMC5983_CONF_A 0x00
#define HMC5983_CONF_B 0x01
#define HMC5983_MODE 0x02

#define HMC5983_OUT_X_MSB 0x03
#define HMC5983_OUT_X_LSB 0x04
#define HMC5983_OUT_Z_MSB 0x05
#define HMC5983_OUT_Z_LSB 0x06
#define HMC5983_OUT_Y_MSB 0x07
#define HMC5983_OUT_Y_LSB 0x08

#define HMC5983_ID_A 0x0A
#define HMC5983_ID_B 0x0B
#define HMC5983_ID_C 0x0C

#define HMC5983_SCALE_1 0.000729927f
#define HMC5983_SCALE_2 0.000917431f
#define HMC5983_SCALE_3 0.001219512f
#define HMC5983_SCALE_4 0.001515152f
#define HMC5983_SCALE_5 0.002272727f
#define HMC5983_SCALE_6 0.002564103f
#define HMC5983_SCALE_7 0.003030303f
#define HMC5983_SCALE_8 0.004347826f

#define hmc5983_chip_select() GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define hmc5983_chip_deselect() GPIO_SetBits(GPIOA, GPIO_Pin_4)

int hmc5983_init();

void hmc5983_read_unscaled_data(vector3d_16_t *mag_unscaled_data);
void hmc5983_mag_convert_to_scale(vector3d_16_t *mag_unscaled_data,
                                  vector3d_f_t *mag_scaled_data);

#endif
