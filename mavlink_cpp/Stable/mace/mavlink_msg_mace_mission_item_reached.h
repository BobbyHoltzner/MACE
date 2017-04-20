#pragma once
// MESSAGE MACE_MISSION_ITEM_REACHED PACKING

#define MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED 309

MAVPACKED(
typedef struct __mavlink_mace_mission_item_reached_t {
 uint16_t seq; /*< Sequence*/
 uint8_t mission_type; /*< Mission type, see MACE_MISSION_PROFILE*/
}) mavlink_mace_mission_item_reached_t;

#define MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN 3
#define MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN 3
#define MAVLINK_MSG_ID_309_LEN 3
#define MAVLINK_MSG_ID_309_MIN_LEN 3

#define MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC 190
#define MAVLINK_MSG_ID_309_CRC 190



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_MISSION_ITEM_REACHED { \
    309, \
    "MACE_MISSION_ITEM_REACHED", \
    2, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mace_mission_item_reached_t, seq) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_mission_item_reached_t, mission_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_MISSION_ITEM_REACHED { \
    "MACE_MISSION_ITEM_REACHED", \
    2, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mace_mission_item_reached_t, seq) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mace_mission_item_reached_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_mission_item_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_mission_item_reached_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN);
#else
    mavlink_mace_mission_item_reached_t packet;
    packet.seq = seq;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
}

/**
 * @brief Pack a mace_mission_item_reached message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_mission_item_reached_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq,uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN);
#else
    mavlink_mace_mission_item_reached_t packet;
    packet.seq = seq;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
}

/**
 * @brief Encode a mace_mission_item_reached struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_mission_item_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_mission_item_reached_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_mission_item_reached_t* mace_mission_item_reached)
{
    return mavlink_msg_mace_mission_item_reached_pack(system_id, component_id, msg, mace_mission_item_reached->seq, mace_mission_item_reached->mission_type);
}

/**
 * @brief Encode a mace_mission_item_reached struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_mission_item_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_mission_item_reached_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_mission_item_reached_t* mace_mission_item_reached)
{
    return mavlink_msg_mace_mission_item_reached_pack_chan(system_id, component_id, chan, msg, mace_mission_item_reached->seq, mace_mission_item_reached->mission_type);
}

/**
 * @brief Send a mace_mission_item_reached message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 * @param mission_type Mission type, see MACE_MISSION_PROFILE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_mission_item_reached_send(mavlink_channel_t chan, uint16_t seq, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
#else
    mavlink_mace_mission_item_reached_t packet;
    packet.seq = seq;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED, (const char *)&packet, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
#endif
}

/**
 * @brief Send a mace_mission_item_reached message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_mission_item_reached_send_struct(mavlink_channel_t chan, const mavlink_mace_mission_item_reached_t* mace_mission_item_reached)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_mission_item_reached_send(chan, mace_mission_item_reached->seq, mace_mission_item_reached->mission_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED, (const char *)mace_mission_item_reached, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_mission_item_reached_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
#else
    mavlink_mace_mission_item_reached_t *packet = (mavlink_mace_mission_item_reached_t *)msgbuf;
    packet->seq = seq;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED, (const char *)packet, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_MIN_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_MISSION_ITEM_REACHED UNPACKING


/**
 * @brief Get field seq from mace_mission_item_reached message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_mace_mission_item_reached_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field mission_type from mace_mission_item_reached message
 *
 * @return Mission type, see MACE_MISSION_PROFILE
 */
static inline uint8_t mavlink_msg_mace_mission_item_reached_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a mace_mission_item_reached message into a struct
 *
 * @param msg The message to decode
 * @param mace_mission_item_reached C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_mission_item_reached_decode(const mavlink_message_t* msg, mavlink_mace_mission_item_reached_t* mace_mission_item_reached)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_mission_item_reached->seq = mavlink_msg_mace_mission_item_reached_get_seq(msg);
    mace_mission_item_reached->mission_type = mavlink_msg_mace_mission_item_reached_get_mission_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN? msg->len : MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN;
        memset(mace_mission_item_reached, 0, MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED_LEN);
    memcpy(mace_mission_item_reached, _MAV_PAYLOAD(msg), len);
#endif
}
