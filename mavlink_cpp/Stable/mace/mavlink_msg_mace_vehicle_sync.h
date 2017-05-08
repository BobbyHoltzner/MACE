#pragma once
// MESSAGE MACE_VEHICLE_SYNC PACKING

#define MAVLINK_MSG_ID_MACE_VEHICLE_SYNC 194

MAVPACKED(
typedef struct __mavlink_mace_vehicle_sync_t {
 uint8_t target_system; /*< System ID*/
}) mavlink_mace_vehicle_sync_t;

#define MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN 1
#define MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN 1
#define MAVLINK_MSG_ID_194_LEN 1
#define MAVLINK_MSG_ID_194_MIN_LEN 1

#define MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC 114
#define MAVLINK_MSG_ID_194_CRC 114



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_VEHICLE_SYNC { \
    194, \
    "MACE_VEHICLE_SYNC", \
    1, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_vehicle_sync_t, target_system) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_VEHICLE_SYNC { \
    "MACE_VEHICLE_SYNC", \
    1, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mace_vehicle_sync_t, target_system) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_vehicle_sync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_vehicle_sync_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN);
#else
    mavlink_mace_vehicle_sync_t packet;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_VEHICLE_SYNC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
}

/**
 * @brief Pack a mace_vehicle_sync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_vehicle_sync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN);
#else
    mavlink_mace_vehicle_sync_t packet;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_VEHICLE_SYNC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
}

/**
 * @brief Encode a mace_vehicle_sync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_vehicle_sync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_vehicle_sync_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_vehicle_sync_t* mace_vehicle_sync)
{
    return mavlink_msg_mace_vehicle_sync_pack(system_id, component_id, msg, mace_vehicle_sync->target_system);
}

/**
 * @brief Encode a mace_vehicle_sync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_vehicle_sync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_vehicle_sync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_vehicle_sync_t* mace_vehicle_sync)
{
    return mavlink_msg_mace_vehicle_sync_pack_chan(system_id, component_id, chan, msg, mace_vehicle_sync->target_system);
}

/**
 * @brief Send a mace_vehicle_sync message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_vehicle_sync_send(mavlink_channel_t chan, uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC, buf, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
#else
    mavlink_mace_vehicle_sync_t packet;
    packet.target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC, (const char *)&packet, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
#endif
}

/**
 * @brief Send a mace_vehicle_sync message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_vehicle_sync_send_struct(mavlink_channel_t chan, const mavlink_mace_vehicle_sync_t* mace_vehicle_sync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_vehicle_sync_send(chan, mace_vehicle_sync->target_system);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC, (const char *)mace_vehicle_sync, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_vehicle_sync_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC, buf, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
#else
    mavlink_mace_vehicle_sync_t *packet = (mavlink_mace_vehicle_sync_t *)msgbuf;
    packet->target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC, (const char *)packet, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_MIN_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_VEHICLE_SYNC UNPACKING


/**
 * @brief Get field target_system from mace_vehicle_sync message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mace_vehicle_sync_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a mace_vehicle_sync message into a struct
 *
 * @param msg The message to decode
 * @param mace_vehicle_sync C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_vehicle_sync_decode(const mavlink_message_t* msg, mavlink_mace_vehicle_sync_t* mace_vehicle_sync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_vehicle_sync->target_system = mavlink_msg_mace_vehicle_sync_get_target_system(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN? msg->len : MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN;
        memset(mace_vehicle_sync, 0, MAVLINK_MSG_ID_MACE_VEHICLE_SYNC_LEN);
    memcpy(mace_vehicle_sync, _MAV_PAYLOAD(msg), len);
#endif
}
