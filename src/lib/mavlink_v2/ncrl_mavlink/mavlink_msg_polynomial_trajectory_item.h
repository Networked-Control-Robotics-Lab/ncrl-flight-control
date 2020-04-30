#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_ITEM PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM 11003

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_item_t {
 float coeff[8]; /*<  Coefficients of the trajectory*/
 float flight_time; /*<  Flight time*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t type; /*<  Type (x, y, z or yaw)*/
 uint8_t index; /*<  Index of trajectory segment*/
}) mavlink_polynomial_trajectory_item_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN 40
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN 40
#define MAVLINK_MSG_ID_11003_LEN 40
#define MAVLINK_MSG_ID_11003_MIN_LEN 40

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC 107
#define MAVLINK_MSG_ID_11003_CRC 107

#define MAVLINK_MSG_POLYNOMIAL_TRAJECTORY_ITEM_FIELD_COEFF_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_ITEM { \
    11003, \
    "POLYNOMIAL_TRAJECTORY_ITEM", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_polynomial_trajectory_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_polynomial_trajectory_item_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_polynomial_trajectory_item_t, type) }, \
         { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_polynomial_trajectory_item_t, index) }, \
         { "coeff", NULL, MAVLINK_TYPE_FLOAT, 8, 0, offsetof(mavlink_polynomial_trajectory_item_t, coeff) }, \
         { "flight_time", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_polynomial_trajectory_item_t, flight_time) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_ITEM { \
    "POLYNOMIAL_TRAJECTORY_ITEM", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_polynomial_trajectory_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_polynomial_trajectory_item_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_polynomial_trajectory_item_t, type) }, \
         { "index", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_polynomial_trajectory_item_t, index) }, \
         { "coeff", NULL, MAVLINK_TYPE_FLOAT, 8, 0, offsetof(mavlink_polynomial_trajectory_item_t, coeff) }, \
         { "flight_time", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_polynomial_trajectory_item_t, flight_time) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param type  Type (x, y, z or yaw)
 * @param index  Index of trajectory segment
 * @param coeff  Coefficients of the trajectory
 * @param flight_time  Flight time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t index, const float *coeff, float flight_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN];
    _mav_put_float(buf, 32, flight_time);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_uint8_t(buf, 39, index);
    _mav_put_float_array(buf, 0, coeff, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN);
#else
    mavlink_polynomial_trajectory_item_t packet;
    packet.flight_time = flight_time;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.index = index;
    mav_array_memcpy(packet.coeff, coeff, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param type  Type (x, y, z or yaw)
 * @param index  Index of trajectory segment
 * @param coeff  Coefficients of the trajectory
 * @param flight_time  Flight time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t index,const float *coeff,float flight_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN];
    _mav_put_float(buf, 32, flight_time);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_uint8_t(buf, 39, index);
    _mav_put_float_array(buf, 0, coeff, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN);
#else
    mavlink_polynomial_trajectory_item_t packet;
    packet.flight_time = flight_time;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.index = index;
    mav_array_memcpy(packet.coeff, coeff, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_item_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_item_t* polynomial_trajectory_item)
{
    return mavlink_msg_polynomial_trajectory_item_pack(system_id, component_id, msg, polynomial_trajectory_item->target_system, polynomial_trajectory_item->target_component, polynomial_trajectory_item->type, polynomial_trajectory_item->index, polynomial_trajectory_item->coeff, polynomial_trajectory_item->flight_time);
}

/**
 * @brief Encode a polynomial_trajectory_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_item_t* polynomial_trajectory_item)
{
    return mavlink_msg_polynomial_trajectory_item_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_item->target_system, polynomial_trajectory_item->target_component, polynomial_trajectory_item->type, polynomial_trajectory_item->index, polynomial_trajectory_item->coeff, polynomial_trajectory_item->flight_time);
}

/**
 * @brief Send a polynomial_trajectory_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param type  Type (x, y, z or yaw)
 * @param index  Index of trajectory segment
 * @param coeff  Coefficients of the trajectory
 * @param flight_time  Flight time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_item_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t index, const float *coeff, float flight_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN];
    _mav_put_float(buf, 32, flight_time);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_uint8_t(buf, 39, index);
    _mav_put_float_array(buf, 0, coeff, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
#else
    mavlink_polynomial_trajectory_item_t packet;
    packet.flight_time = flight_time;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.index = index;
    mav_array_memcpy(packet.coeff, coeff, sizeof(float)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_item message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_item_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_item_t* polynomial_trajectory_item)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_item_send(chan, polynomial_trajectory_item->target_system, polynomial_trajectory_item->target_component, polynomial_trajectory_item->type, polynomial_trajectory_item->index, polynomial_trajectory_item->coeff, polynomial_trajectory_item->flight_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM, (const char *)polynomial_trajectory_item, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_item_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t index, const float *coeff, float flight_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 32, flight_time);
    _mav_put_uint8_t(buf, 36, target_system);
    _mav_put_uint8_t(buf, 37, target_component);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_uint8_t(buf, 39, index);
    _mav_put_float_array(buf, 0, coeff, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
#else
    mavlink_polynomial_trajectory_item_t *packet = (mavlink_polynomial_trajectory_item_t *)msgbuf;
    packet->flight_time = flight_time;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->type = type;
    packet->index = index;
    mav_array_memcpy(packet->coeff, coeff, sizeof(float)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_ITEM UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_item message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_item_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field target_component from polynomial_trajectory_item message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_item_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field type from polynomial_trajectory_item message
 *
 * @return  Type (x, y, z or yaw)
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_item_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field index from polynomial_trajectory_item message
 *
 * @return  Index of trajectory segment
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_item_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field coeff from polynomial_trajectory_item message
 *
 * @return  Coefficients of the trajectory
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_item_get_coeff(const mavlink_message_t* msg, float *coeff)
{
    return _MAV_RETURN_float_array(msg, coeff, 8,  0);
}

/**
 * @brief Get field flight_time from polynomial_trajectory_item message
 *
 * @return  Flight time
 */
static inline float mavlink_msg_polynomial_trajectory_item_get_flight_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a polynomial_trajectory_item message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_item C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_item_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_item_t* polynomial_trajectory_item)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_item_get_coeff(msg, polynomial_trajectory_item->coeff);
    polynomial_trajectory_item->flight_time = mavlink_msg_polynomial_trajectory_item_get_flight_time(msg);
    polynomial_trajectory_item->target_system = mavlink_msg_polynomial_trajectory_item_get_target_system(msg);
    polynomial_trajectory_item->target_component = mavlink_msg_polynomial_trajectory_item_get_target_component(msg);
    polynomial_trajectory_item->type = mavlink_msg_polynomial_trajectory_item_get_type(msg);
    polynomial_trajectory_item->index = mavlink_msg_polynomial_trajectory_item_get_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN;
        memset(polynomial_trajectory_item, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_LEN);
    memcpy(polynomial_trajectory_item, _MAV_PAYLOAD(msg), len);
#endif
}
