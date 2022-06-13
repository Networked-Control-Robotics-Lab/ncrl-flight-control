#ifndef __PROJ_CONFIG_H__
#define __PROJ_CONFIG_H__

/* uav info */
#define UAV_DEFAULT_ID 1 //vaild from 1~250 according to qgroundcontrol

/* uav type */
#define UAV_TYPE_QUADROTOR 1
#define SELECT_UAV_TYPE UAV_TYPE_QUADROTOR

/* flight control board (automatically set by makefile) */
#define BOARD_PROTOTYPE_V1 1
#define BOARD_PX4_V246     2

/*===========================*
 * telemetry system settings *
 *===========================*/

/* telemetry main channel protocols */
#define TELEM_MAVLINK    1
#define SELECT_MAIN_TELEM TELEM_MAVLINK

/* telemetry debug channel protocols */
#define TELEM_SHELL      1
#define TELEM_DEBUG_LINK 2
#define SELECT_DEBUG_TELEM TELEM_DEBUG_LINK

/* debug link message publish rate */
#define DEBUG_LINK_PUBLISH_20Hz  1 //recommanded for wireless communication
#define DEBUG_LINK_PUBLISH_100Hz 2 //recommanded for wired communication
#define DEBUG_LINK_PUBLISH_RATE DEBUG_LINK_PUBLISH_20Hz

/*==========================*
 * state estimator settings *
 *==========================*/

/* ahrs algorithms */
#define AHRS_COMPLEMENTARY_FILTER 1
#define AHRS_MADGWICK_FILTER      2
#define AHRS_ESKF                 3
#define AHRS_OPTITRACK            4
#define SELECT_AHRS AHRS_ESKF

/* ins algorithms */
#define INS_COMPLEMENTARY_FILTER 1
#define INS_ESKF                 2
#define SELECT_INS INS_ESKF

/* quadrotor control algorithms */
#define QUADROTOR_USE_PID      1
#define QUADROTOR_USE_GEOMETRY 2
#define SELECT_CONTROLLER QUADROTOR_USE_GEOMETRY

/*===================*
 * hardware settings *
 *===================*/

/* navigation device 1 */
#define NAV_DEV1_NO_CONNECTION 1
#define NAV_DEV1_USE_OPTITRACK 2
#define NAV_DEV1_USE_GPS       3
#define SELECT_NAVIGATION_DEVICE1 NAV_DEV1_USE_OPTITRACK

/* navigation device 2 */
#define NAV_DEV2_NO_CONNECTION 1
#define NAV_DEV2_USE_VINS_MONO 2
#define SELECT_NAVIGATION_DEVICE2 NAV_DEV2_NO_CONNECTION

/* compass sensor option */
#define ENABLE_MAGNETOMETER    0

/* barometer sensor option */
#define ENABLE_BAROMETER       0

/* rangefinder sensor option */
#define ENABLE_RANGEFINDER     0

/*=======================================================*
 * sensor source settings for state estimation algorithm *
 *=======================================================*/

/* heading fusion soruce */
#define NO_HEADING_SENSOR            1
#define HEADING_FUSION_USE_COMPASS   2
#define HEADING_FUSION_USE_OPTITRACK 3
#define HEADING_FUSION_USE_VINS_MONO 4
#define SELECT_HEADING_SENSOR HEADING_FUSION_USE_OPTITRACK

/* height fusion source */
#define NO_HEIGHT_SENSOR              1
#define HEIGHT_FUSION_USE_BAROMETER   2
#define HEIGHT_FUSION_USE_OPTITRACK   3
#define HEIGHT_FUSION_USE_VINS_MONO   4
#define HEIGHT_FUSION_USE_RANGEFINDER 5
#define SELECT_HEIGHT_SENSOR HEIGHT_FUSION_USE_OPTITRACK

/* position fusion source */
#define NO_POSITION_SENSOR            1
#define POSITION_FUSION_USE_GPS       2
#define POSITION_FUSION_USE_OPTITRACK 3
#define POSITION_FUSION_USE_VINS_MONO 4
#define SELECT_POSITION_SENSOR POSITION_FUSION_USE_OPTITRACK

/* configuration validation */
#if (SELECT_NAVIGATION_DEVICE1 != NAV_DEV1_USE_GPS) && (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_GPS)
#error "gps receiver is not enabled."
#endif

#if (SELECT_NAVIGATION_DEVICE1 != NAV_DEV1_USE_OPTITRACK) && \
    ((SELECT_HEADING_SENSOR == HEADING_FUSION_USE_OPTITRACK) || \
     (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_OPTITRACK) || \
     (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_OPTITRACK))
#error "optitrack is not enabled."
#endif

#if (SELECT_NAVIGATION_DEVICE2 != NAV_DEV2_USE_VINS_MONO) && \
    ((SELECT_HEADING_SENSOR == HEADING_FUSION_USE_VINS_MONO) || \
     (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_VINS_MONO) || \
     (SELECT_POSITION_SENSOR == POSITION_FUSION_USE_VINS_MONO))
#error "vins-mono is not enabled."
#endif

#if (ENABLE_MAGNETOMETER == 0) && (SELECT_HEADING_SENSOR == HEADING_FUSION_USE_COMPASS)
#error "magnetometer is not enabled."
#endif

#if (ENABLE_BAROMETER == 0) && (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_BAROMETER)
#error "barometer is not enabled."
#endif

#if (ENABLE_RANGEFINDER == 0) && (SELECT_HEIGHT_SENSOR == HEIGHT_FUSION_USE_RANGEFINDER)
#error "rangefinder is not enabled."
#endif

#endif
