#pragma once
// MESSAGE MACE_ACK_MISSION PACKING

#define MAVLINK_MSG_ID_MACE_ACK_MISSION 183

MAVPACKED(
typedef struct __mavlink_mace_ack_mission_t {
 uint8_t target_system; /*< System ID*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MACE_MISSION_TYPE*/
 uint8_t mission_state; /*< Mission type, see MACE_MISSION_STATE*/
}) mavlink_mace_ack_mission_t;

#define MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN 6
#define MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN 6
#define MAVLINK_MSG_ID_183_LEN 6
#define MAVLINK_MSG_ID_183_MIN_LEN 6

#define MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC 50
#define MAVLINK_MSG_ID_183_CRC 50



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_ACK_MISSION { \
    183, \
    "MACE_ACK_MISSION", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_ack_mission_t, target_system) }, \
         { "mission_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_ack_mission_t, mission_system) }, \
         { "mission_creator", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_ack_mission_t, mission_creator) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_ack_mission_t, mission_id) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mace_ack_mission_t, mission_type) }, \
         { "mission_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mace_ack_mission_t, mission_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_ACK_MISSION { \
    "MACE_ACK_MISSION", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_ack_mission_t, target_system) }, \
         { "mission_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_ack_mission_t, mission_system) }, \
         { "mission_creator", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_ack_mission_t, mission_creator) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_ack_mission_t, mission_id) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mace_ack_mission_t, mission_type) }, \
         { "mission_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mace_ack_mission_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_ack_mission message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 * @param mission_state Mission type, see MACE_MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_ack_mission_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, mission_system);
    _mav_put_uint8_t(buf, 2, mission_creator);
    _mav_put_uint8_t(buf, 3, mission_id);
    _mav_put_uint8_t(buf, 4, mission_type);
    _mav_put_uint8_t(buf, 5, mission_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN);
#else
    mavlink_mace_ack_mission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_ACK_MISSION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
}

/**
 * @brief Pack a mace_ack_mission message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 * @param mission_state Mission type, see MACE_MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_ack_mission_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, mission_system);
    _mav_put_uint8_t(buf, 2, mission_creator);
    _mav_put_uint8_t(buf, 3, mission_id);
    _mav_put_uint8_t(buf, 4, mission_type);
    _mav_put_uint8_t(buf, 5, mission_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN);
#else
    mavlink_mace_ack_mission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_ACK_MISSION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
}

/**
 * @brief Encode a mace_ack_mission struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_ack_mission C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_ack_mission_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_ack_mission_t* mace_ack_mission)
{
    return mavlink_msg_mace_ack_mission_pack(system_id, component_id, msg, mace_ack_mission->target_system, mace_ack_mission->mission_system, mace_ack_mission->mission_creator, mace_ack_mission->mission_id, mace_ack_mission->mission_type, mace_ack_mission->mission_state);
}

/**
 * @brief Encode a mace_ack_mission struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_ack_mission C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_ack_mission_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_ack_mission_t* mace_ack_mission)
{
    return mavlink_msg_mace_ack_mission_pack_chan(system_id, component_id, chan, msg, mace_ack_mission->target_system, mace_ack_mission->mission_system, mace_ack_mission->mission_creator, mace_ack_mission->mission_id, mace_ack_mission->mission_type, mace_ack_mission->mission_state);
}

/**
 * @brief Send a mace_ack_mission message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 * @param mission_state Mission type, see MACE_MISSION_STATE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_ack_mission_send(mavlink_channel_t chan, uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, mission_system);
    _mav_put_uint8_t(buf, 2, mission_creator);
    _mav_put_uint8_t(buf, 3, mission_id);
    _mav_put_uint8_t(buf, 4, mission_type);
    _mav_put_uint8_t(buf, 5, mission_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ACK_MISSION, buf, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
#else
    mavlink_mace_ack_mission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ACK_MISSION, (const char *)&packet, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
#endif
}

/**
 * @brief Send a mace_ack_mission message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_ack_mission_send_struct(mavlink_channel_t chan, const mavlink_mace_ack_mission_t* mace_ack_mission)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_ack_mission_send(chan, mace_ack_mission->target_system, mace_ack_mission->mission_system, mace_ack_mission->mission_creator, mace_ack_mission->mission_id, mace_ack_mission->mission_type, mace_ack_mission->mission_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ACK_MISSION, (const char *)mace_ack_mission, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_ack_mission_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, mission_system);
    _mav_put_uint8_t(buf, 2, mission_creator);
    _mav_put_uint8_t(buf, 3, mission_id);
    _mav_put_uint8_t(buf, 4, mission_type);
    _mav_put_uint8_t(buf, 5, mission_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ACK_MISSION, buf, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
#else
    mavlink_mace_ack_mission_t *packet = (mavlink_mace_ack_mission_t *)msgbuf;
    packet->target_system = target_system;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ACK_MISSION, (const char *)packet, MAVLINK_MSG_ID_MACE_ACK_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN, MAVLINK_MSG_ID_MACE_ACK_MISSION_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_ACK_MISSION UNPACKING


/**
 * @brief Get field target_system from mace_ack_mission message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_system from mace_ack_mission message
 *
 * @return Mission System ID
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_mission_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_creator from mace_ack_mission message
 *
 * @return Creator ID
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_mission_creator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_id from mace_ack_mission message
 *
 * @return Mission ID
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_type from mace_ack_mission message
 *
 * @return Mission type, see MACE_MISSION_TYPE
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_state from mace_ack_mission message
 *
 * @return Mission type, see MACE_MISSION_STATE
 */
static inline uint8_t mavlink_msg_mace_ack_mission_get_mission_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a mace_ack_mission message into a struct
 *
 * @param msg The message to decode
 * @param mace_ack_mission C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_ack_mission_decode(const mavlink_message_t* msg, mavlink_mace_ack_mission_t* mace_ack_mission)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_ack_mission->target_system = mavlink_msg_mace_ack_mission_get_target_system(msg);
    mace_ack_mission->mission_system = mavlink_msg_mace_ack_mission_get_mission_system(msg);
    mace_ack_mission->mission_creator = mavlink_msg_mace_ack_mission_get_mission_creator(msg);
    mace_ack_mission->mission_id = mavlink_msg_mace_ack_mission_get_mission_id(msg);
    mace_ack_mission->mission_type = mavlink_msg_mace_ack_mission_get_mission_type(msg);
    mace_ack_mission->mission_state = mavlink_msg_mace_ack_mission_get_mission_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN? msg->len : MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN;
        memset(mace_ack_mission, 0, MAVLINK_MSG_ID_MACE_ACK_MISSION_LEN);
    memcpy(mace_ack_mission, _MAV_PAYLOAD(msg), len);
#endif
}
