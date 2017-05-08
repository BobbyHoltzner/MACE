#pragma once
// MESSAGE MACE_STARTING_CURRENT_MISSION PACKING

#define MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION 180

MAVPACKED(
typedef struct __mavlink_mace_starting_current_mission_t {
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MACE_MISSION_TYPE*/
}) mavlink_mace_starting_current_mission_t;

#define MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN 4
#define MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN 4
#define MAVLINK_MSG_ID_180_LEN 4
#define MAVLINK_MSG_ID_180_MIN_LEN 4

#define MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC 179
#define MAVLINK_MSG_ID_180_CRC 179



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_STARTING_CURRENT_MISSION { \
    180, \
    "MACE_STARTING_CURRENT_MISSION", \
    4, \
    {  { "mission_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_starting_current_mission_t, mission_system) }, \
         { "mission_creator", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_starting_current_mission_t, mission_creator) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_starting_current_mission_t, mission_id) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_starting_current_mission_t, mission_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_STARTING_CURRENT_MISSION { \
    "MACE_STARTING_CURRENT_MISSION", \
    4, \
    {  { "mission_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_starting_current_mission_t, mission_system) }, \
         { "mission_creator", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mace_starting_current_mission_t, mission_creator) }, \
         { "mission_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_starting_current_mission_t, mission_id) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mace_starting_current_mission_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_starting_current_mission message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_starting_current_mission_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, mission_system);
    _mav_put_uint8_t(buf, 1, mission_creator);
    _mav_put_uint8_t(buf, 2, mission_id);
    _mav_put_uint8_t(buf, 3, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN);
#else
    mavlink_mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
}

/**
 * @brief Pack a mace_starting_current_mission message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_starting_current_mission_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, mission_system);
    _mav_put_uint8_t(buf, 1, mission_creator);
    _mav_put_uint8_t(buf, 2, mission_id);
    _mav_put_uint8_t(buf, 3, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN);
#else
    mavlink_mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
}

/**
 * @brief Encode a mace_starting_current_mission struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_starting_current_mission C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_starting_current_mission_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_starting_current_mission_t* mace_starting_current_mission)
{
    return mavlink_msg_mace_starting_current_mission_pack(system_id, component_id, msg, mace_starting_current_mission->mission_system, mace_starting_current_mission->mission_creator, mace_starting_current_mission->mission_id, mace_starting_current_mission->mission_type);
}

/**
 * @brief Encode a mace_starting_current_mission struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_starting_current_mission C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_starting_current_mission_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_starting_current_mission_t* mace_starting_current_mission)
{
    return mavlink_msg_mace_starting_current_mission_pack_chan(system_id, component_id, chan, msg, mace_starting_current_mission->mission_system, mace_starting_current_mission->mission_creator, mace_starting_current_mission->mission_id, mace_starting_current_mission->mission_type);
}

/**
 * @brief Send a mace_starting_current_mission message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MACE_MISSION_TYPE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_starting_current_mission_send(mavlink_channel_t chan, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN];
    _mav_put_uint8_t(buf, 0, mission_system);
    _mav_put_uint8_t(buf, 1, mission_creator);
    _mav_put_uint8_t(buf, 2, mission_id);
    _mav_put_uint8_t(buf, 3, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION, buf, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
#else
    mavlink_mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION, (const char *)&packet, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
#endif
}

/**
 * @brief Send a mace_starting_current_mission message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_starting_current_mission_send_struct(mavlink_channel_t chan, const mavlink_mace_starting_current_mission_t* mace_starting_current_mission)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_starting_current_mission_send(chan, mace_starting_current_mission->mission_system, mace_starting_current_mission->mission_creator, mace_starting_current_mission->mission_id, mace_starting_current_mission->mission_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION, (const char *)mace_starting_current_mission, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_starting_current_mission_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, mission_system);
    _mav_put_uint8_t(buf, 1, mission_creator);
    _mav_put_uint8_t(buf, 2, mission_id);
    _mav_put_uint8_t(buf, 3, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION, buf, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
#else
    mavlink_mace_starting_current_mission_t *packet = (mavlink_mace_starting_current_mission_t *)msgbuf;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION, (const char *)packet, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_MIN_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_STARTING_CURRENT_MISSION UNPACKING


/**
 * @brief Get field mission_system from mace_starting_current_mission message
 *
 * @return Mission System ID
 */
static inline uint8_t mavlink_msg_mace_starting_current_mission_get_mission_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_creator from mace_starting_current_mission message
 *
 * @return Creator ID
 */
static inline uint8_t mavlink_msg_mace_starting_current_mission_get_mission_creator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_id from mace_starting_current_mission message
 *
 * @return Mission ID
 */
static inline uint8_t mavlink_msg_mace_starting_current_mission_get_mission_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from mace_starting_current_mission message
 *
 * @return Mission type, see MACE_MISSION_TYPE
 */
static inline uint8_t mavlink_msg_mace_starting_current_mission_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mace_starting_current_mission message into a struct
 *
 * @param msg The message to decode
 * @param mace_starting_current_mission C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_starting_current_mission_decode(const mavlink_message_t* msg, mavlink_mace_starting_current_mission_t* mace_starting_current_mission)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_starting_current_mission->mission_system = mavlink_msg_mace_starting_current_mission_get_mission_system(msg);
    mace_starting_current_mission->mission_creator = mavlink_msg_mace_starting_current_mission_get_mission_creator(msg);
    mace_starting_current_mission->mission_id = mavlink_msg_mace_starting_current_mission_get_mission_id(msg);
    mace_starting_current_mission->mission_type = mavlink_msg_mace_starting_current_mission_get_mission_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN? msg->len : MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN;
        memset(mace_starting_current_mission, 0, MAVLINK_MSG_ID_MACE_STARTING_CURRENT_MISSION_LEN);
    memcpy(mace_starting_current_mission, _MAV_PAYLOAD(msg), len);
#endif
}
