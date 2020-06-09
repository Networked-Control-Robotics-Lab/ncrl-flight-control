#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_POSITION_DEBUG PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG 11004

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_position_debug_t {
 float x; /*<  uav current x position*/
 float y; /*<  uav current y position*/
 float z; /*<  uav current z position*/
 float x_d; /*<  uav current desired x position*/
 float y_d; /*<  uav current desired y position*/
 float z_d; /*<  uav current desired z position*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
}) mavlink_polynomial_trajectory_position_debug_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN 26
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN 26
#define MAVLINK_MSG_ID_11004_LEN 26
#define MAVLINK_MSG_ID_11004_MIN_LEN 26

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC 206
#define MAVLINK_MSG_ID_11004_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG { \
    11004, \
    "POLYNOMIAL_TRAJECTORY_POSITION_DEBUG", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_polynomial_trajectory_position_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_polynomial_trajectory_position_debug_t, target_component) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_position_debug_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_position_debug_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_position_debug_t, z) }, \
         { "x_d", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_polynomial_trajectory_position_debug_t, x_d) }, \
         { "y_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_polynomial_trajectory_position_debug_t, y_d) }, \
         { "z_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_polynomial_trajectory_position_debug_t, z_d) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG { \
    "POLYNOMIAL_TRAJECTORY_POSITION_DEBUG", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_polynomial_trajectory_position_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_polynomial_trajectory_position_debug_t, target_component) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_position_debug_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_position_debug_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_position_debug_t, z) }, \
         { "x_d", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_polynomial_trajectory_position_debug_t, x_d) }, \
         { "y_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_polynomial_trajectory_position_debug_t, y_d) }, \
         { "z_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_polynomial_trajectory_position_debug_t, z_d) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_position_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param x  uav current x position
 * @param y  uav current y position
 * @param z  uav current z position
 * @param x_d  uav current desired x position
 * @param y_d  uav current desired y position
 * @param z_d  uav current desired z position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_position_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float x, float y, float z, float x_d, float y_d, float z_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, x_d);
    _mav_put_float(buf, 16, y_d);
    _mav_put_float(buf, 20, z_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_position_debug_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.x_d = x_d;
    packet.y_d = y_d;
    packet.z_d = z_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_position_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param x  uav current x position
 * @param y  uav current y position
 * @param z  uav current z position
 * @param x_d  uav current desired x position
 * @param y_d  uav current desired y position
 * @param z_d  uav current desired z position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_position_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float x,float y,float z,float x_d,float y_d,float z_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, x_d);
    _mav_put_float(buf, 16, y_d);
    _mav_put_float(buf, 20, z_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_position_debug_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.x_d = x_d;
    packet.y_d = y_d;
    packet.z_d = z_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_position_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_position_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_position_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_position_debug_t* polynomial_trajectory_position_debug)
{
    return mavlink_msg_polynomial_trajectory_position_debug_pack(system_id, component_id, msg, polynomial_trajectory_position_debug->target_system, polynomial_trajectory_position_debug->target_component, polynomial_trajectory_position_debug->x, polynomial_trajectory_position_debug->y, polynomial_trajectory_position_debug->z, polynomial_trajectory_position_debug->x_d, polynomial_trajectory_position_debug->y_d, polynomial_trajectory_position_debug->z_d);
}

/**
 * @brief Encode a polynomial_trajectory_position_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_position_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_position_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_position_debug_t* polynomial_trajectory_position_debug)
{
    return mavlink_msg_polynomial_trajectory_position_debug_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_position_debug->target_system, polynomial_trajectory_position_debug->target_component, polynomial_trajectory_position_debug->x, polynomial_trajectory_position_debug->y, polynomial_trajectory_position_debug->z, polynomial_trajectory_position_debug->x_d, polynomial_trajectory_position_debug->y_d, polynomial_trajectory_position_debug->z_d);
}

/**
 * @brief Send a polynomial_trajectory_position_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param x  uav current x position
 * @param y  uav current y position
 * @param z  uav current z position
 * @param x_d  uav current desired x position
 * @param y_d  uav current desired y position
 * @param z_d  uav current desired z position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_position_debug_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float x, float y, float z, float x_d, float y_d, float z_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, x_d);
    _mav_put_float(buf, 16, y_d);
    _mav_put_float(buf, 20, z_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_position_debug_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.x_d = x_d;
    packet.y_d = y_d;
    packet.z_d = z_d;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_position_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_position_debug_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_position_debug_t* polynomial_trajectory_position_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_position_debug_send(chan, polynomial_trajectory_position_debug->target_system, polynomial_trajectory_position_debug->target_component, polynomial_trajectory_position_debug->x, polynomial_trajectory_position_debug->y, polynomial_trajectory_position_debug->z, polynomial_trajectory_position_debug->x_d, polynomial_trajectory_position_debug->y_d, polynomial_trajectory_position_debug->z_d);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG, (const char *)polynomial_trajectory_position_debug, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_position_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float x, float y, float z, float x_d, float y_d, float z_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, x_d);
    _mav_put_float(buf, 16, y_d);
    _mav_put_float(buf, 20, z_d);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_position_debug_t *packet = (mavlink_polynomial_trajectory_position_debug_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->x_d = x_d;
    packet->y_d = y_d;
    packet->z_d = z_d;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_POSITION_DEBUG UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_position_debug message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_position_debug_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from polynomial_trajectory_position_debug message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_position_debug_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field x from polynomial_trajectory_position_debug message
 *
 * @return  uav current x position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from polynomial_trajectory_position_debug message
 *
 * @return  uav current y position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from polynomial_trajectory_position_debug message
 *
 * @return  uav current z position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x_d from polynomial_trajectory_position_debug message
 *
 * @return  uav current desired x position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_x_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y_d from polynomial_trajectory_position_debug message
 *
 * @return  uav current desired y position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_y_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z_d from polynomial_trajectory_position_debug message
 *
 * @return  uav current desired z position
 */
static inline float mavlink_msg_polynomial_trajectory_position_debug_get_z_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a polynomial_trajectory_position_debug message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_position_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_position_debug_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_position_debug_t* polynomial_trajectory_position_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    polynomial_trajectory_position_debug->x = mavlink_msg_polynomial_trajectory_position_debug_get_x(msg);
    polynomial_trajectory_position_debug->y = mavlink_msg_polynomial_trajectory_position_debug_get_y(msg);
    polynomial_trajectory_position_debug->z = mavlink_msg_polynomial_trajectory_position_debug_get_z(msg);
    polynomial_trajectory_position_debug->x_d = mavlink_msg_polynomial_trajectory_position_debug_get_x_d(msg);
    polynomial_trajectory_position_debug->y_d = mavlink_msg_polynomial_trajectory_position_debug_get_y_d(msg);
    polynomial_trajectory_position_debug->z_d = mavlink_msg_polynomial_trajectory_position_debug_get_z_d(msg);
    polynomial_trajectory_position_debug->target_system = mavlink_msg_polynomial_trajectory_position_debug_get_target_system(msg);
    polynomial_trajectory_position_debug->target_component = mavlink_msg_polynomial_trajectory_position_debug_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN;
        memset(polynomial_trajectory_position_debug, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_LEN);
    memcpy(polynomial_trajectory_position_debug, _MAV_PAYLOAD(msg), len);
#endif
}
