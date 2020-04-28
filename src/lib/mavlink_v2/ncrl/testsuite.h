/** @file
 *    @brief MAVLink comm protocol testsuite generated from ncrl.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef NCRL_TESTSUITE_H
#define NCRL_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ncrl(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_ncrl(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_polynomial_trajectory_write(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_write_t packet_in = {
        5,72,139
    };
    mavlink_polynomial_trajectory_write_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.list_size = packet_in.list_size;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_WRITE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_write_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_write_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.list_size );
    mavlink_msg_polynomial_trajectory_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_write_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.list_size );
    mavlink_msg_polynomial_trajectory_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_write_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.list_size );
    mavlink_msg_polynomial_trajectory_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_cmd_t packet_in = {
        5,72,139,206
    };
    mavlink_polynomial_trajectory_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.cmd = packet_in.cmd;
        packet1.option = packet_in.option;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_cmd_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.cmd , packet1.option );
    mavlink_msg_polynomial_trajectory_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.cmd , packet1.option );
    mavlink_msg_polynomial_trajectory_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_cmd_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.cmd , packet1.option );
    mavlink_msg_polynomial_trajectory_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_ack_t packet_in = {
        5,72,139
    };
    mavlink_polynomial_trajectory_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.ack_val = packet_in.ack_val;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.ack_val );
    mavlink_msg_polynomial_trajectory_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.ack_val );
    mavlink_msg_polynomial_trajectory_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.ack_val );
    mavlink_msg_polynomial_trajectory_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_item(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_item_t packet_in = {
        { 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0 },{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0 },{ 465.0, 466.0, 467.0, 468.0, 469.0, 470.0, 471.0, 472.0 },{ 689.0, 690.0, 691.0, 692.0, 693.0, 694.0, 695.0, 696.0 },133,200
    };
    mavlink_polynomial_trajectory_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.x_coeff, packet_in.x_coeff, sizeof(float)*8);
        mav_array_memcpy(packet1.y_coeff, packet_in.y_coeff, sizeof(float)*8);
        mav_array_memcpy(packet1.z_coeff, packet_in.z_coeff, sizeof(float)*8);
        mav_array_memcpy(packet1.yaw_coeff, packet_in.yaw_coeff, sizeof(float)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_item_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.x_coeff , packet1.y_coeff , packet1.z_coeff , packet1.yaw_coeff );
    mavlink_msg_polynomial_trajectory_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_item_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.x_coeff , packet1.y_coeff , packet1.z_coeff , packet1.yaw_coeff );
    mavlink_msg_polynomial_trajectory_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_item_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.x_coeff , packet1.y_coeff , packet1.z_coeff , packet1.yaw_coeff );
    mavlink_msg_polynomial_trajectory_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ncrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_polynomial_trajectory_write(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_cmd(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_ack(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_item(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // NCRL_TESTSUITE_H
