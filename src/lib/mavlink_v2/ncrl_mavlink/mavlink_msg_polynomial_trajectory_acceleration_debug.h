#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG 11006

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_acceleration_debug_t {
 float ax_ff; /*<  uav current desired x acceleration feedforward*/
 float ay_ff; /*<  uav current desired y acceleration feedforward*/
 float az_ff; /*<  uav current desired z acceleration feedforward*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
}) mavlink_polynomial_trajectory_acceleration_debug_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN 14
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN 14
#define MAVLINK_MSG_ID_11006_LEN 14
#define MAVLINK_MSG_ID_11006_MIN_LEN 14

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC 96
#define MAVLINK_MSG_ID_11006_CRC 96



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG { \
    11006, \
    "POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, target_component) }, \
         { "ax_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, ax_ff) }, \
         { "ay_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, ay_ff) }, \
         { "az_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, az_ff) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG { \
    "POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, target_component) }, \
         { "ax_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, ax_ff) }, \
         { "ay_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, ay_ff) }, \
         { "az_ff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_polynomial_trajectory_acceleration_debug_t, az_ff) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_acceleration_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ax_ff  uav current desired x acceleration feedforward
 * @param ay_ff  uav current desired y acceleration feedforward
 * @param az_ff  uav current desired z acceleration feedforward
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_acceleration_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float ax_ff, float ay_ff, float az_ff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN];
    _mav_put_float(buf, 0, ax_ff);
    _mav_put_float(buf, 4, ay_ff);
    _mav_put_float(buf, 8, az_ff);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_acceleration_debug_t packet;
    packet.ax_ff = ax_ff;
    packet.ay_ff = ay_ff;
    packet.az_ff = az_ff;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_acceleration_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ax_ff  uav current desired x acceleration feedforward
 * @param ay_ff  uav current desired y acceleration feedforward
 * @param az_ff  uav current desired z acceleration feedforward
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_acceleration_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float ax_ff,float ay_ff,float az_ff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN];
    _mav_put_float(buf, 0, ax_ff);
    _mav_put_float(buf, 4, ay_ff);
    _mav_put_float(buf, 8, az_ff);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN);
#else
    mavlink_polynomial_trajectory_acceleration_debug_t packet;
    packet.ax_ff = ax_ff;
    packet.ay_ff = ay_ff;
    packet.az_ff = az_ff;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_acceleration_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_acceleration_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_acceleration_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_acceleration_debug_t* polynomial_trajectory_acceleration_debug)
{
    return mavlink_msg_polynomial_trajectory_acceleration_debug_pack(system_id, component_id, msg, polynomial_trajectory_acceleration_debug->target_system, polynomial_trajectory_acceleration_debug->target_component, polynomial_trajectory_acceleration_debug->ax_ff, polynomial_trajectory_acceleration_debug->ay_ff, polynomial_trajectory_acceleration_debug->az_ff);
}

/**
 * @brief Encode a polynomial_trajectory_acceleration_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_acceleration_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_acceleration_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_acceleration_debug_t* polynomial_trajectory_acceleration_debug)
{
    return mavlink_msg_polynomial_trajectory_acceleration_debug_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_acceleration_debug->target_system, polynomial_trajectory_acceleration_debug->target_component, polynomial_trajectory_acceleration_debug->ax_ff, polynomial_trajectory_acceleration_debug->ay_ff, polynomial_trajectory_acceleration_debug->az_ff);
}

/**
 * @brief Send a polynomial_trajectory_acceleration_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ax_ff  uav current desired x acceleration feedforward
 * @param ay_ff  uav current desired y acceleration feedforward
 * @param az_ff  uav current desired z acceleration feedforward
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_acceleration_debug_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float ax_ff, float ay_ff, float az_ff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN];
    _mav_put_float(buf, 0, ax_ff);
    _mav_put_float(buf, 4, ay_ff);
    _mav_put_float(buf, 8, az_ff);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_acceleration_debug_t packet;
    packet.ax_ff = ax_ff;
    packet.ay_ff = ay_ff;
    packet.az_ff = az_ff;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_acceleration_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_acceleration_debug_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_acceleration_debug_t* polynomial_trajectory_acceleration_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_acceleration_debug_send(chan, polynomial_trajectory_acceleration_debug->target_system, polynomial_trajectory_acceleration_debug->target_component, polynomial_trajectory_acceleration_debug->ax_ff, polynomial_trajectory_acceleration_debug->ay_ff, polynomial_trajectory_acceleration_debug->az_ff);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG, (const char *)polynomial_trajectory_acceleration_debug, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_acceleration_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float ax_ff, float ay_ff, float az_ff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, ax_ff);
    _mav_put_float(buf, 4, ay_ff);
    _mav_put_float(buf, 8, az_ff);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
#else
    mavlink_polynomial_trajectory_acceleration_debug_t *packet = (mavlink_polynomial_trajectory_acceleration_debug_t *)msgbuf;
    packet->ax_ff = ax_ff;
    packet->ay_ff = ay_ff;
    packet->az_ff = az_ff;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_acceleration_debug message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_acceleration_debug_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from polynomial_trajectory_acceleration_debug message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_acceleration_debug_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field ax_ff from polynomial_trajectory_acceleration_debug message
 *
 * @return  uav current desired x acceleration feedforward
 */
static inline float mavlink_msg_polynomial_trajectory_acceleration_debug_get_ax_ff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ay_ff from polynomial_trajectory_acceleration_debug message
 *
 * @return  uav current desired y acceleration feedforward
 */
static inline float mavlink_msg_polynomial_trajectory_acceleration_debug_get_ay_ff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field az_ff from polynomial_trajectory_acceleration_debug message
 *
 * @return  uav current desired z acceleration feedforward
 */
static inline float mavlink_msg_polynomial_trajectory_acceleration_debug_get_az_ff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a polynomial_trajectory_acceleration_debug message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_acceleration_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_acceleration_debug_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_acceleration_debug_t* polynomial_trajectory_acceleration_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    polynomial_trajectory_acceleration_debug->ax_ff = mavlink_msg_polynomial_trajectory_acceleration_debug_get_ax_ff(msg);
    polynomial_trajectory_acceleration_debug->ay_ff = mavlink_msg_polynomial_trajectory_acceleration_debug_get_ay_ff(msg);
    polynomial_trajectory_acceleration_debug->az_ff = mavlink_msg_polynomial_trajectory_acceleration_debug_get_az_ff(msg);
    polynomial_trajectory_acceleration_debug->target_system = mavlink_msg_polynomial_trajectory_acceleration_debug_get_target_system(msg);
    polynomial_trajectory_acceleration_debug->target_component = mavlink_msg_polynomial_trajectory_acceleration_debug_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN;
        memset(polynomial_trajectory_acceleration_debug, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_LEN);
    memcpy(polynomial_trajectory_acceleration_debug, _MAV_PAYLOAD(msg), len);
#endif
}
