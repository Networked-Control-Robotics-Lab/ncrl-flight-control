#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG 11005

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_velocity_debug_t {
 float vx; /*<  uav current x velocity*/
 float vy; /*<  uav current y velocity*/
 float vz; /*<  uav current z velocity*/
 float vx_d; /*<  uav current desired x velocity*/
 float vy_d; /*<  uav current desired y velocity*/
 float vz_d; /*<  uav current desired z velocity*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
}) mavlink_polynomial_trajectory_velocity_debug_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN 26
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN 26
#define MAVLINK_MSG_ID_11005_LEN 26
#define MAVLINK_MSG_ID_11005_MIN_LEN 26

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC 0
#define MAVLINK_MSG_ID_11005_CRC 0



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG { \
    11005, \
    "POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vz) }, \
         { "vx_d", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vx_d) }, \
         { "vy_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vy_d) }, \
         { "vz_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vz_d) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG { \
    "POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, target_component) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vz) }, \
         { "vx_d", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vx_d) }, \
         { "vy_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vy_d) }, \
         { "vz_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_polynomial_trajectory_velocity_debug_t, vz_d) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_velocity_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param vx  uav current x velocity
 * @param vy  uav current y velocity
 * @param vz  uav current z velocity
 * @param vx_d  uav current desired x velocity
 * @param vy_d  uav current desired y velocity
 * @param vz_d  uav current desired z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_velocity_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float vx_d, float vy_d, float vz_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, vx_d);
    _mav_put_float(buf, 16, vy_d);
    _mav_put_float(buf, 20, vz_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_velocity_debug_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vx_d = vx_d;
    packet.vy_d = vy_d;
    packet.vz_d = vz_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_velocity_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param vx  uav current x velocity
 * @param vy  uav current y velocity
 * @param vz  uav current z velocity
 * @param vx_d  uav current desired x velocity
 * @param vy_d  uav current desired y velocity
 * @param vz_d  uav current desired z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_velocity_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float vx,float vy,float vz,float vx_d,float vy_d,float vz_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, vx_d);
    _mav_put_float(buf, 16, vy_d);
    _mav_put_float(buf, 20, vz_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_velocity_debug_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vx_d = vx_d;
    packet.vy_d = vy_d;
    packet.vz_d = vz_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_velocity_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_velocity_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_velocity_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_velocity_debug_t* polynomial_trajectory_velocity_debug)
{
    return mavlink_msg_polynomial_trajectory_velocity_debug_pack(system_id, component_id, msg, polynomial_trajectory_velocity_debug->target_system, polynomial_trajectory_velocity_debug->target_component, polynomial_trajectory_velocity_debug->vx, polynomial_trajectory_velocity_debug->vy, polynomial_trajectory_velocity_debug->vz, polynomial_trajectory_velocity_debug->vx_d, polynomial_trajectory_velocity_debug->vy_d, polynomial_trajectory_velocity_debug->vz_d);
}

/**
 * @brief Encode a polynomial_trajectory_velocity_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_velocity_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_velocity_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_velocity_debug_t* polynomial_trajectory_velocity_debug)
{
    return mavlink_msg_polynomial_trajectory_velocity_debug_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_velocity_debug->target_system, polynomial_trajectory_velocity_debug->target_component, polynomial_trajectory_velocity_debug->vx, polynomial_trajectory_velocity_debug->vy, polynomial_trajectory_velocity_debug->vz, polynomial_trajectory_velocity_debug->vx_d, polynomial_trajectory_velocity_debug->vy_d, polynomial_trajectory_velocity_debug->vz_d);
}

/**
 * @brief Send a polynomial_trajectory_velocity_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param vx  uav current x velocity
 * @param vy  uav current y velocity
 * @param vz  uav current z velocity
 * @param vx_d  uav current desired x velocity
 * @param vy_d  uav current desired y velocity
 * @param vz_d  uav current desired z velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_velocity_debug_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float vx_d, float vy_d, float vz_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, vx_d);
    _mav_put_float(buf, 16, vy_d);
    _mav_put_float(buf, 20, vz_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_velocity_debug_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vx_d = vx_d;
    packet.vy_d = vy_d;
    packet.vz_d = vz_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_velocity_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_velocity_debug_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_velocity_debug_t* polynomial_trajectory_velocity_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_velocity_debug_send(chan, polynomial_trajectory_velocity_debug->target_system, polynomial_trajectory_velocity_debug->target_component, polynomial_trajectory_velocity_debug->vx, polynomial_trajectory_velocity_debug->vy, polynomial_trajectory_velocity_debug->vz, polynomial_trajectory_velocity_debug->vx_d, polynomial_trajectory_velocity_debug->vy_d, polynomial_trajectory_velocity_debug->vz_d);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG, (const char *)polynomial_trajectory_velocity_debug, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_velocity_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float vx, float vy, float vz, float vx_d, float vy_d, float vz_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, vx_d);
    _mav_put_float(buf, 16, vy_d);
    _mav_put_float(buf, 20, vz_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_velocity_debug_t *packet = (mavlink_polynomial_trajectory_velocity_debug_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->vx_d = vx_d;
    packet->vy_d = vy_d;
    packet->vz_d = vz_d;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_velocity_debug message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_velocity_debug_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from polynomial_trajectory_velocity_debug message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_velocity_debug_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field vx from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current x velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current y velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vz from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current z velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vx_d from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current desired x velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vx_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vy_d from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current desired y velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vy_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vz_d from polynomial_trajectory_velocity_debug message
 *
 * @return  uav current desired z velocity
 */
static inline float mavlink_msg_polynomial_trajectory_velocity_debug_get_vz_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a polynomial_trajectory_velocity_debug message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_velocity_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_velocity_debug_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_velocity_debug_t* polynomial_trajectory_velocity_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    polynomial_trajectory_velocity_debug->vx = mavlink_msg_polynomial_trajectory_velocity_debug_get_vx(msg);
    polynomial_trajectory_velocity_debug->vy = mavlink_msg_polynomial_trajectory_velocity_debug_get_vy(msg);
    polynomial_trajectory_velocity_debug->vz = mavlink_msg_polynomial_trajectory_velocity_debug_get_vz(msg);
    polynomial_trajectory_velocity_debug->vx_d = mavlink_msg_polynomial_trajectory_velocity_debug_get_vx_d(msg);
    polynomial_trajectory_velocity_debug->vy_d = mavlink_msg_polynomial_trajectory_velocity_debug_get_vy_d(msg);
    polynomial_trajectory_velocity_debug->vz_d = mavlink_msg_polynomial_trajectory_velocity_debug_get_vz_d(msg);
    polynomial_trajectory_velocity_debug->target_system = mavlink_msg_polynomial_trajectory_velocity_debug_get_target_system(msg);
    polynomial_trajectory_velocity_debug->target_component = mavlink_msg_polynomial_trajectory_velocity_debug_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN;
        memset(polynomial_trajectory_velocity_debug, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_LEN);
    memcpy(polynomial_trajectory_velocity_debug, _MAV_PAYLOAD(msg), len);
#endif
}
