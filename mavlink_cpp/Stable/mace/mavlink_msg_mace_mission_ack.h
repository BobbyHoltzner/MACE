#pragma once
// MESSAGE MACE_MISSION_ACK PACKING

#define MAVLINK_MSG_ID_MACE_MISSION_ACK 310

MAVPACKED(
typedef struct __mavlink_mace_mission_ack_t {
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t type; /*< See MAV_MISSION_RESULT enum*/
 uint8_t mission_type; /*< Mission type, see MACE_MISSION_PROFILE*/
}) mavlink_mace_mission_ack_t;

#define MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN 4
#define MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN 4
#define MAVLINK_MSG_ID_310_LEN 4
#define MAVLINK_MSG_ID_310_MIN_LEN 4

#define MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC 22
#define MAVLINK_MSG_ID_310_CRC 22



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_MISSION_ACK { \
    310, \
    "MACE_MISSION_ACK", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_mission_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_mission_ack_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_mission_ack_t, type) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_mission_ack_t, mission_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_MISSION_ACK { \
    "MACE_MISSION_ACK", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_mission_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_mission_ack_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_mission_ack_t, type) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_mission_ack_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_mission_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN);
#else
    mavlink_mace_mission_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_MISSION_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
}

/**
 * @brief Pack a mace_mission_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_mission_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN);
#else
    mavlink_mace_mission_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_MISSION_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
}

/**
 * @brief Encode a mace_mission_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_mission_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_mission_ack_t* mace_mission_ack)
{
    return mavlink_msg_mace_mission_ack_pack(system_id, component_id, msg, mace_mission_ack->target_system, mace_mission_ack->target_component, mace_mission_ack->type, mace_mission_ack->mission_type);
}

/**
 * @brief Encode a mace_mission_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_mission_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_mission_ack_t* mace_mission_ack)
{
    return mavlink_msg_mace_mission_ack_pack_chan(system_id, component_id, chan, msg, mace_mission_ack->target_system, mace_mission_ack->target_component, mace_mission_ack->type, mace_mission_ack->mission_type);
}

/**
 * @brief Send a mace_mission_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_mission_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ACK, buf, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
#else
    mavlink_mace_mission_ack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.type = type;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ACK, (const char *)&packet, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
#endif
}

/**
 * @brief Send a mace_mission_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_mission_ack_send_struct(mavlink_channel_t chan, const mavlink_mace_mission_ack_t* mace_mission_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_mission_ack_send(chan, mace_mission_ack->target_system, mace_mission_ack->target_component, mace_mission_ack->type, mace_mission_ack->mission_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ACK, (const char *)mace_mission_ack, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_mission_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, type);
    _mav_put_uint8_t(buf, 3, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ACK, buf, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
#else
    mavlink_mace_mission_ack_t *packet = (mavlink_mace_mission_ack_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->type = type;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ACK, (const char *)packet, MAVLINK_MSG_ID_MACE_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN, MAVLINK_MSG_ID_MACE_MISSION_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_MISSION_ACK UNPACKING


/**
 * @brief Get field target_system from mace_mission_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mace_mission_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mace_mission_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mace_mission_ack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field type from mace_mission_ack message
 *
 * @return See MAV_MISSION_RESULT enum
 */
static inline uint8_t mavlink_msg_mace_mission_ack_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from mace_mission_ack message
 *
 * @return Mission type, see MACE_MISSION_PROFILE
 */
static inline uint8_t mavlink_msg_mace_mission_ack_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mace_mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param mace_mission_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_mission_ack_decode(const mavlink_message_t* msg, mavlink_mace_mission_ack_t* mace_mission_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_mission_ack->target_system = mavlink_msg_mace_mission_ack_get_target_system(msg);
    mace_mission_ack->target_component = mavlink_msg_mace_mission_ack_get_target_component(msg);
    mace_mission_ack->type = mavlink_msg_mace_mission_ack_get_type(msg);
    mace_mission_ack->mission_type = mavlink_msg_mace_mission_ack_get_mission_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN? msg->len : MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN;
        memset(mace_mission_ack, 0, MAVLINK_MSG_ID_MACE_MISSION_ACK_LEN);
    memcpy(mace_mission_ack, _MAV_PAYLOAD(msg), len);
#endif
}
