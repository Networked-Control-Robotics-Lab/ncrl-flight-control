#pragma once
// MESSAGE POLYNOMIAL_TRAJECTORY_CMD PACKING

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD 11001

MAVPACKED(
typedef struct __mavlink_polynomial_trajectory_cmd_t {
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t cmd; /*<  Polynomial trajectory command*/
 uint8_t option; /*<  Polynomial trajectory command option*/
}) mavlink_polynomial_trajectory_cmd_t;

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN 4
#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN 4
#define MAVLINK_MSG_ID_11001_LEN 4
#define MAVLINK_MSG_ID_11001_MIN_LEN 4

#define MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC 236
#define MAVLINK_MSG_ID_11001_CRC 236



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_CMD { \
    11001, \
    "POLYNOMIAL_TRAJECTORY_CMD", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_polynomial_trajectory_cmd_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_polynomial_trajectory_cmd_t, target_component) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_polynomial_trajectory_cmd_t, cmd) }, \
         { "option", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_polynomial_trajectory_cmd_t, option) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POLYNOMIAL_TRAJECTORY_CMD { \
    "POLYNOMIAL_TRAJECTORY_CMD", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_polynomial_trajectory_cmd_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_polynomial_trajectory_cmd_t, target_component) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_polynomial_trajectory_cmd_t, cmd) }, \
         { "option", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_polynomial_trajectory_cmd_t, option) }, \
         } \
}
#endif

/**
 * @brief Pack a polynomial_trajectory_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param cmd  Polynomial trajectory command
 * @param option  Polynomial trajectory command option
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t cmd, uint8_t option)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, option);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN);
#else
    mavlink_polynomial_trajectory_cmd_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd = cmd;
    packet.option = option;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
}

/**
 * @brief Pack a polynomial_trajectory_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param cmd  Polynomial trajectory command
 * @param option  Polynomial trajectory command option
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t cmd,uint8_t option)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, option);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN);
#else
    mavlink_polynomial_trajectory_cmd_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd = cmd;
    packet.option = option;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
}

/**
 * @brief Encode a polynomial_trajectory_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_polynomial_trajectory_cmd_t* polynomial_trajectory_cmd)
{
    return mavlink_msg_polynomial_trajectory_cmd_pack(system_id, component_id, msg, polynomial_trajectory_cmd->target_system, polynomial_trajectory_cmd->target_component, polynomial_trajectory_cmd->cmd, polynomial_trajectory_cmd->option);
}

/**
 * @brief Encode a polynomial_trajectory_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param polynomial_trajectory_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_polynomial_trajectory_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_polynomial_trajectory_cmd_t* polynomial_trajectory_cmd)
{
    return mavlink_msg_polynomial_trajectory_cmd_pack_chan(system_id, component_id, chan, msg, polynomial_trajectory_cmd->target_system, polynomial_trajectory_cmd->target_component, polynomial_trajectory_cmd->cmd, polynomial_trajectory_cmd->option);
}

/**
 * @brief Send a polynomial_trajectory_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param cmd  Polynomial trajectory command
 * @param option  Polynomial trajectory command option
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_polynomial_trajectory_cmd_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t cmd, uint8_t option)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, option);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
#else
    mavlink_polynomial_trajectory_cmd_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd = cmd;
    packet.option = option;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD, (const char *)&packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
#endif
}

/**
 * @brief Send a polynomial_trajectory_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_polynomial_trajectory_cmd_send_struct(mavlink_channel_t chan, const mavlink_polynomial_trajectory_cmd_t* polynomial_trajectory_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_polynomial_trajectory_cmd_send(chan, polynomial_trajectory_cmd->target_system, polynomial_trajectory_cmd->target_component, polynomial_trajectory_cmd->cmd, polynomial_trajectory_cmd->option);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD, (const char *)polynomial_trajectory_cmd, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_polynomial_trajectory_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t cmd, uint8_t option)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, option);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD, buf, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
#else
    mavlink_polynomial_trajectory_cmd_t *packet = (mavlink_polynomial_trajectory_cmd_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->cmd = cmd;
    packet->option = option;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD, (const char *)packet, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE POLYNOMIAL_TRAJECTORY_CMD UNPACKING


/**
 * @brief Get field target_system from polynomial_trajectory_cmd message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_cmd_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from polynomial_trajectory_cmd message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_cmd_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field cmd from polynomial_trajectory_cmd message
 *
 * @return  Polynomial trajectory command
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_cmd_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field option from polynomial_trajectory_cmd message
 *
 * @return  Polynomial trajectory command option
 */
static inline uint8_t mavlink_msg_polynomial_trajectory_cmd_get_option(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a polynomial_trajectory_cmd message into a struct
 *
 * @param msg The message to decode
 * @param polynomial_trajectory_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_polynomial_trajectory_cmd_decode(const mavlink_message_t* msg, mavlink_polynomial_trajectory_cmd_t* polynomial_trajectory_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    polynomial_trajectory_cmd->target_system = mavlink_msg_polynomial_trajectory_cmd_get_target_system(msg);
    polynomial_trajectory_cmd->target_component = mavlink_msg_polynomial_trajectory_cmd_get_target_component(msg);
    polynomial_trajectory_cmd->cmd = mavlink_msg_polynomial_trajectory_cmd_get_cmd(msg);
    polynomial_trajectory_cmd->option = mavlink_msg_polynomial_trajectory_cmd_get_option(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN? msg->len : MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN;
        memset(polynomial_trajectory_cmd, 0, MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_LEN);
    memcpy(polynomial_trajectory_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
