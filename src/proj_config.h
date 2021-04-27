#ifndef __PROJ_CONFIG_H__
#define __PROJ_CONFIG_H__

/* uav info */
#define UAV_DEFAULT_ID 1 //vaild from 1~250 according to qgroundcontrol

/* uav type */
#define UAV_TYPE_QUADROTOR 0
#define SELECT_UAV_TYPE UAV_TYPE_QUADROTOR

/* telemetry main channel protocols */
#define TELEM_MAVLINK    0
#define SELECT_MAIN_TELEM TELEM_MAVLINK

/* telemetry debug channel protocols */
#define TELEM_SHELL      0
#define TELEM_DEBUG_LINK 1
#define SELECT_DEBUG_TELEM TELEM_SHELL

/* debug link message publish rate */
#define DEBUG_LINK_PUBLISH_20Hz  0 //recommanded for wireless communication
#define DEBUG_LINK_PUBLISH_100Hz 1 //recommanded for wired communication
#define DEBUG_LINK_PUBLISH_RATE DEBUG_LINK_PUBLISH_20Hz

/* ahrs algorithms */
#define AHRS_COMPLEMENTARY_FILTER 0
#define AHRS_MADGWICK_FILTER      1
#define AHRS_ESKF                 2
#define AHRS_OPTITRACK            3 
#define SELECT_AHRS AHRS_ESKF

/* ins algorithms */
#define INS_COMPLEMENTARY_FILTER 0
#define INS_ESKF                 1
#define SELECT_INS INS_COMPLEMENTARY_FILTER

/* quadrotor control algorithms */
#define QUADROTOR_USE_PID      0
#define QUADROTOR_USE_GEOMETRY 1
#define SELECT_CONTROLLER QUADROTOR_USE_GEOMETRY

/* heading sensors */
#define NO_HEADING_SENSOR            0
#define HEADING_SENSOR_USE_COMPASS   1
#define HEADING_SENSOR_USE_OPTITRACK 2
#define HEADING_SENSOR_USE_VINS_MONO 3
#define SELECT_HEADING_SENSOR HEADING_SENSOR_USE_OPTITRACK

/* height sensors */
#define NO_HEIGHT_SENSOR            0
#define HEIGHT_SENSOR_USE_BAROMETER 1
#define HEIGHT_SENSOR_USE_OPTITRACK 2
#define HEIGHT_SENSOR_USE_VINS_MONO 3
#define SELECT_HEIGHT_SENSOR HEIGHT_SENSOR_USE_OPTITRACK

/* position sensors */
#define NO_POSITION_SENSOR            0
#define POSITION_SENSOR_USE_GPS       1
#define POSITION_SENSOR_USE_OPTITRACK 2
#define POSITION_SENSOR_USE_VINS_MONO 3
#define SELECT_POSITION_SENSOR POSITION_SENSOR_USE_OPTITRACK

#endif
