#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_WRITE PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE 11000

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_write_t {
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t list_size; /*<  Trajectory list size*/
 uint8_t z_enabled; /*<  UAV should track z trajectory (1: true, 0: false)*/
 uint8_t yaw_enabled; /*<  UAV should track yaw trajectory (1: true, 0: false)*/
}) mavlink_polynomial_trajectory_write_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN 5
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN 5
#define MAVLINK_MSG_ID_11000_LEN 5
#define MAVLINK_MSG_ID_11000_MIN_LEN 5

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC 43
#define MAVLINK_MSG_ID_11000_CRC 43



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_WRITE { \
    11000, \
    "POLYNOMIAL_TRAJECTORY_WRITE", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_polynomial_trajectory_write_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_polynomial_trajectory_write_t, target_component) }, \
         { "list_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_polynomial_trajectory_write_t, list_size) }, \
         { "z_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_polynomial_trajectory_write_t, z_enabled) }, \
         { "yaw_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_polynomial_trajectory_write_t, yaw_enabled) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_WRITE { \
    "POLYNOMIAL_TRAJECTORY_WRITE", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_polynomial_trajectory_write_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_polynomial_trajectory_write_t, target_component) }, \
         { "list_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_polynomial_trajectory_write_t, list_size) }, \
         { "z_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_polynomial_trajectory_write_t, z_enabled) }, \
         { "yaw_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_polynomial_trajectory_write_t, yaw_enabled) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_write message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param list_size  Trajectory list size
 * @param z_enabled  UAV should track z trajectory (1: true, 0: false)
 * @param yaw_enabled  UAV should track yaw trajectory (1: true, 0: false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_write_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t list_size, uint8_t z_enabled, uint8_t yaw_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, list_size);
    _mav_put_uint8_t(buf, 3, z_enabled);
    _mav_put_uint8_t(buf, 4, yaw_enabled);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN);
#else
    mavlink_polynomial_trajectory_write_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.list_size = list_size;
    packet.z_enabled = z_enabled;
    packet.yaw_enabled = yaw_enabled;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_write message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param list_size  Trajectory list size
 * @param z_enabled  UAV should track z trajectory (1: true, 0: false)
 * @param yaw_enabled  UAV should track yaw trajectory (1: true, 0: false)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_write_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t list_size,uint8_t z_enabled,uint8_t yaw_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, list_size);
    _mav_put_uint8_t(buf, 3, z_enabled);
    _mav_put_uint8_t(buf, 4, yaw_enabled);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN);
#else
    mavlink_polynomial_trajectory_write_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.list_size = list_size;
    packet.z_enabled = z_enabled;
    packet.yaw_enabled = yaw_enabled;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_write struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_write C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_write_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_write_t* polynomial_trajectory_write)
{
    return mavlink_msg_polynomial_trajectory_write_pack(system_id, component_id, msg, polynomial_trajectory_write->target_system, polynomial_trajectory_write->target_component, polynomial_trajectory_write->list_size, polynomial_trajectory_write->z_enabled, polynomial_trajectory_write->yaw_enabled);
}

/**
 * @brief Encode a polynomial_trajectory_write struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_write C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_write_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_write_t* polynomial_trajectory_write)
{
    return mavlink_msg_polynomial_trajectory_write_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_write->target_system, polynomial_trajectory_write->target_component, polynomial_trajectory_write->list_size, polynomial_trajectory_write->z_enabled, polynomial_trajectory_write->yaw_enabled);
}

/**
 * @brief Send a polynomial_trajectory_write message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param list_size  Trajectory list size
 * @param z_enabled  UAV should track z trajectory (1: true, 0: false)
 * @param yaw_enabled  UAV should track yaw trajectory (1: true, 0: false)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_write_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t list_size, uint8_t z_enabled, uint8_t yaw_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, list_size);
    _mav_put_uint8_t(buf, 3, z_enabled);
    _mav_put_uint8_t(buf, 4, yaw_enabled);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
#else
    mavlink_polynomial_trajectory_write_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.list_size = list_size;
    packet.z_enabled = z_enabled;
    packet.yaw_enabled = yaw_enabled;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_write message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_write_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_write_t* polynomial_trajectory_write)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_write_send(chan, polynomial_trajectory_write->target_system, polynomial_trajectory_write->target_component, polynomial_trajectory_write->list_size, polynomial_trajectory_write->z_enabled, polynomial_trajectory_write->yaw_enabled);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE, (const char *)polynomial_trajectory_write, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_write_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t list_size, uint8_t z_enabled, uint8_t yaw_enabled)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, list_size);
    _mav_put_uint8_t(buf, 3, z_enabled);
    _mav_put_uint8_t(buf, 4, yaw_enabled);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
#else
    mavlink_polynomial_trajectory_write_t *packet = (mavlink_polynomial_trajectory_write_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->list_size = list_size;
    packet->z_enabled = z_enabled;
    packet->yaw_enabled = yaw_enabled;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_WRITE UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_write message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_write_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from polynomial_trajectory_write message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_write_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field list_size from polynomial_trajectory_write message
 *
 * @return  Trajectory list size
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_write_get_list_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field z_enabled from polynomial_trajectory_write message
 *
 * @return  UAV should track z trajectory (1: true, 0: false)
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_write_get_z_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field yaw_enabled from polynomial_trajectory_write message
 *
 * @return  UAV should track yaw trajectory (1: true, 0: false)
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_write_get_yaw_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a polynomial_trajectory_write message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_write C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_write_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_write_t* polynomial_trajectory_write)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    polynomial_trajectory_write->target_system = mavlink_msg_polynomial_trajectory_write_get_target_system(msg);
    polynomial_trajectory_write->target_component = mavlink_msg_polynomial_trajectory_write_get_target_component(msg);
    polynomial_trajectory_write->list_size = mavlink_msg_polynomial_trajectory_write_get_list_size(msg);
    polynomial_trajectory_write->z_enabled = mavlink_msg_polynomial_trajectory_write_get_z_enabled(msg);
    polynomial_trajectory_write->yaw_enabled = mavlink_msg_polynomial_trajectory_write_get_yaw_enabled(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN;
        memset(polynomial_trajectory_write, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_LEN);
    memcpy(polynomial_trajectory_write, _MAV_PAYLOAD(msg), len);
#endif
}
