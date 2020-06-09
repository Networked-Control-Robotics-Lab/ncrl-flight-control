/** @file
 *    @brief MAVLink comm protocol testsuite generated from ncrl_mavlink.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef NCRL_MAVLINK_TESTSUITE_H
#define NCRL_MAVLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ncrl_mavlink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_ncrl_mavlink(system_id, component_id, last_msg);
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
        5,72,139,206,17
    };
    mavlink_polynomial_trajectory_write_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.list_size = packet_in.list_size;
        packet1.z_enabled = packet_in.z_enabled;
        packet1.yaw_enabled = packet_in.yaw_enabled;
        
        
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
    mavlink_msg_polynomial_trajectory_write_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.list_size , packet1.z_enabled , packet1.yaw_enabled );
    mavlink_msg_polynomial_trajectory_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_write_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.list_size , packet1.z_enabled , packet1.yaw_enabled );
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
    mavlink_msg_polynomial_trajectory_write_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.list_size , packet1.z_enabled , packet1.yaw_enabled );
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
        { 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0 },241.0,113,180,247,58
    };
    mavlink_polynomial_trajectory_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flight_time = packet_in.flight_time;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.type = packet_in.type;
        packet1.index = packet_in.index;
        
        mav_array_memcpy(packet1.coeff, packet_in.coeff, sizeof(float)*8);
        
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
    mavlink_msg_polynomial_trajectory_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.index , packet1.coeff , packet1.flight_time );
    mavlink_msg_polynomial_trajectory_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_item_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.index , packet1.coeff , packet1.flight_time );
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
    mavlink_msg_polynomial_trajectory_item_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.type , packet1.index , packet1.coeff , packet1.flight_time );
    mavlink_msg_polynomial_trajectory_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_position_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_position_debug_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77,144
    };
    mavlink_polynomial_trajectory_position_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.x_d = packet_in.x_d;
        packet1.y_d = packet_in.y_d;
        packet1.z_d = packet_in.z_d;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_POSITION_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_position_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_position_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_position_debug_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.x_d , packet1.y_d , packet1.z_d );
    mavlink_msg_polynomial_trajectory_position_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_position_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.x_d , packet1.y_d , packet1.z_d );
    mavlink_msg_polynomial_trajectory_position_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_position_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_position_debug_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.x_d , packet1.y_d , packet1.z_d );
    mavlink_msg_polynomial_trajectory_position_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_velocity_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_velocity_debug_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77,144
    };
    mavlink_polynomial_trajectory_velocity_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.vx_d = packet_in.vx_d;
        packet1.vy_d = packet_in.vy_d;
        packet1.vz_d = packet_in.vz_d;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_VELOCITY_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_velocity_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_velocity_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_velocity_debug_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.vx , packet1.vy , packet1.vz , packet1.vx_d , packet1.vy_d , packet1.vz_d );
    mavlink_msg_polynomial_trajectory_velocity_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_velocity_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.vx , packet1.vy , packet1.vz , packet1.vx_d , packet1.vy_d , packet1.vz_d );
    mavlink_msg_polynomial_trajectory_velocity_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_velocity_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_velocity_debug_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.vx , packet1.vy , packet1.vz , packet1.vx_d , packet1.vy_d , packet1.vz_d );
    mavlink_msg_polynomial_trajectory_velocity_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_polynomial_trajectory_acceleration_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_polynomial_trajectory_acceleration_debug_t packet_in = {
        17.0,45.0,73.0,41,108
    };
    mavlink_polynomial_trajectory_acceleration_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ax_ff = packet_in.ax_ff;
        packet1.ay_ff = packet_in.ay_ff;
        packet1.az_ff = packet_in.az_ff;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POLYNOMIAL_TRAJECTORY_ACCELERATION_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_acceleration_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_polynomial_trajectory_acceleration_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_acceleration_debug_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.ax_ff , packet1.ay_ff , packet1.az_ff );
    mavlink_msg_polynomial_trajectory_acceleration_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_acceleration_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.ax_ff , packet1.ay_ff , packet1.az_ff );
    mavlink_msg_polynomial_trajectory_acceleration_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_polynomial_trajectory_acceleration_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_polynomial_trajectory_acceleration_debug_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.ax_ff , packet1.ay_ff , packet1.az_ff );
    mavlink_msg_polynomial_trajectory_acceleration_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ncrl_mavlink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_polynomial_trajectory_write(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_cmd(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_ack(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_item(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_position_debug(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_velocity_debug(system_id, component_id, last_msg);
    mavlink_test_polynomial_trajectory_acceleration_debug(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // NCRL_MAVLINK_TESTSUITE_H
