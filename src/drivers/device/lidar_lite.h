#ifndef __LIDAR_LITE_H__
#define __LIDAR_LITE_H__

#define LIDAR_DEV_ADDRESS 0x62

#define LIDAR_ACQ_COMMAND_REG    0x00
#define LIDAR_STATUS_REG         0x01
#define LIDAR_ACQ_CONFIG_REG     0x04
#define LIDAR_VELOCITY_REG       0x09
#define LIDAR_MEASURE_COUNT_REG  0x11
#define LIDAR_MEASURE_DELAY_REG  0x45
#define LIDAR_DISTANCE_REG       0x8f

void lidar_lite_init(void);

#endif
