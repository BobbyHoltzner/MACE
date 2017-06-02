/** @file
 *    @brief MAVLink comm protocol testsuite generated from mission.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef MISSION_TESTSUITE_H
#define MISSION_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL
static void mace_test_common(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_mission(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_common(system_id, component_id, last_msg);
    mace_test_mission(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mace_test_new_onboard_mission(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_NEW_ONBOARD_MISSION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_new_onboard_mission_t packet_in = {
        5,72,139,206,17
    };
    mace_new_onboard_mission_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.mission_state = packet_in.mission_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_NEW_ONBOARD_MISSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_NEW_ONBOARD_MISSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_onboard_mission_encode(system_id, component_id, &msg, &packet1);
    mace_msg_new_onboard_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_onboard_mission_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_new_onboard_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_onboard_mission_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_new_onboard_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_new_onboard_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_onboard_mission_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_new_onboard_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_new_proposed_mission(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_NEW_PROPOSED_MISSION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_new_proposed_mission_t packet_in = {
        17235,139,206,17,84,151
    };
    mace_new_proposed_mission_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.count = packet_in.count;
        packet1.target_system = packet_in.target_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.mission_state = packet_in.mission_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_proposed_mission_encode(system_id, component_id, &msg, &packet1);
    mace_msg_new_proposed_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_proposed_mission_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_new_proposed_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_proposed_mission_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_new_proposed_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_new_proposed_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_proposed_mission_send(MACE_COMM_1 , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_new_proposed_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_ack_rxmission(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ACK_RXMISSION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_ack_rxmission_t packet_in = {
        5,72,139,206,17,84
    };
    mace_ack_rxmission_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.mission_state = packet_in.mission_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ACK_RXMISSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ACK_RXMISSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ack_rxmission_encode(system_id, component_id, &msg, &packet1);
    mace_msg_ack_rxmission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ack_rxmission_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_ack_rxmission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ack_rxmission_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_ack_rxmission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_ack_rxmission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ack_rxmission_send(MACE_COMM_1 , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_ack_rxmission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_request_list(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_request_list_t packet_in = {
        5,72,139,206
    };
    mace_mission_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_list_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_list_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_list_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_list_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_count(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_COUNT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_count_t packet_in = {
        17235,139,206,17,84,151,218
    };
    mace_mission_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.count = packet_in.count;
        packet1.target_system = packet_in.target_system;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.mission_state = packet_in.mission_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_COUNT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_COUNT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_count_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_count_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_mission_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_count_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_mission_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_count_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_count_send(MACE_COMM_1 , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state , packet1.count );
    mace_msg_mission_count_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_request_item(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_REQUEST_ITEM >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_request_item_t packet_in = {
        17235,139,206,17,84,151
    };
    mace_mission_request_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_item_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_item_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_request_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_item_send(MACE_COMM_1 , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_request_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_item(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_ITEM >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_item_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,18795,101,168,235,46,113,180,247,58
    };
    mace_mission_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.seq = packet_in.seq;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.frame = packet_in.frame;
        packet1.current = packet_in.current;
        packet1.autocontinue = packet_in.autocontinue;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_mission_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_mission_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_send(MACE_COMM_1 , packet1.target_system , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_mission_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_request_partial_list(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_request_partial_list_t packet_in = {
        17235,17339,17,84,151
    };
    mace_mission_request_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start_index = packet_in.start_index;
        packet1.end_index = packet_in.end_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_partial_list_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_request_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_request_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_partial_list_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_request_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_request_partial_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_request_partial_list_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_request_partial_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_write_partial_list(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_WRITE_PARTIAL_LIST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_write_partial_list_t packet_in = {
        17235,17339,17,84,151
    };
    mace_mission_write_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start_index = packet_in.start_index;
        packet1.end_index = packet_in.end_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_write_partial_list_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_write_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_write_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_write_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_write_partial_list_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_write_partial_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_write_partial_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_write_partial_list_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mace_msg_mission_write_partial_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_starting_current_mission(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_STARTING_CURRENT_MISSION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_starting_current_mission_t packet_in = {
        5,72,139,206
    };
    mace_starting_current_mission_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_starting_current_mission_encode(system_id, component_id, &msg, &packet1);
    mace_msg_starting_current_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_starting_current_mission_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_starting_current_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_starting_current_mission_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_starting_current_mission_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_starting_current_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_starting_current_mission_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_starting_current_mission_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_set_current(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_SET_CURRENT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_set_current_t packet_in = {
        17235,139,206,17
    };
    mace_mission_set_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_set_current_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_set_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_set_current_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.seq );
    mace_msg_mission_set_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_set_current_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.seq );
    mace_msg_mission_set_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_set_current_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_set_current_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.seq );
    mace_msg_mission_set_current_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_current(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_CURRENT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_current_t packet_in = {
        17235,139,206,17,84
    };
    mace_mission_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_CURRENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_CURRENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_current_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_current_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_current_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_current_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_current_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_current_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_current_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_item_reached(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_ITEM_REACHED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_item_reached_t packet_in = {
        17235,139,206,17,84
    };
    mace_mission_item_reached_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_ITEM_REACHED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_ITEM_REACHED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_reached_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_item_reached_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_reached_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_item_reached_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_reached_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_item_reached_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_item_reached_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_item_reached_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.seq );
    mace_msg_mission_item_reached_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_clear(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_CLEAR >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_clear_t packet_in = {
        5,72,139,206
    };
    mace_mission_clear_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_CLEAR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_CLEAR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_clear_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_clear_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_clear_pack(system_id, component_id, &msg , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_clear_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_clear_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_clear_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_clear_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_clear_send(MACE_COMM_1 , packet1.target_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type );
    mace_msg_mission_clear_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission_exe_state(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MISSION_EXE_STATE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_mission_exe_state_t packet_in = {
        5,72,139,206,17
    };
    mace_mission_exe_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mission_system = packet_in.mission_system;
        packet1.mission_creator = packet_in.mission_creator;
        packet1.mission_id = packet_in.mission_id;
        packet1.mission_type = packet_in.mission_type;
        packet1.mission_state = packet_in.mission_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_exe_state_encode(system_id, component_id, &msg, &packet1);
    mace_msg_mission_exe_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_exe_state_pack(system_id, component_id, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_mission_exe_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_exe_state_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_mission_exe_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_mission_exe_state_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_mission_exe_state_send(MACE_COMM_1 , packet1.mission_system , packet1.mission_creator , packet1.mission_id , packet1.mission_type , packet1.mission_state );
    mace_msg_mission_exe_state_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mission(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_new_onboard_mission(system_id, component_id, last_msg);
    mace_test_new_proposed_mission(system_id, component_id, last_msg);
    mace_test_ack_rxmission(system_id, component_id, last_msg);
    mace_test_mission_request_list(system_id, component_id, last_msg);
    mace_test_mission_count(system_id, component_id, last_msg);
    mace_test_mission_request_item(system_id, component_id, last_msg);
    mace_test_mission_item(system_id, component_id, last_msg);
    mace_test_mission_request_partial_list(system_id, component_id, last_msg);
    mace_test_mission_write_partial_list(system_id, component_id, last_msg);
    mace_test_starting_current_mission(system_id, component_id, last_msg);
    mace_test_mission_set_current(system_id, component_id, last_msg);
    mace_test_mission_current(system_id, component_id, last_msg);
    mace_test_mission_item_reached(system_id, component_id, last_msg);
    mace_test_mission_clear(system_id, component_id, last_msg);
    mace_test_mission_exe_state(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MISSION_TESTSUITE_H