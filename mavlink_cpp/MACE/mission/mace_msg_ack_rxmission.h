#pragma once
// MESSAGE ACK_RXMISSION PACKING

#define MACE_MSG_ID_ACK_RXMISSION 102

MACEPACKED(
typedef struct __mace_ack_rxmission_t {
 uint8_t target_system; /*< System ID*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t mission_state; /*< The potential new mission state, see MISSION_STATE*/
}) mace_ack_rxmission_t;

#define MACE_MSG_ID_ACK_RXMISSION_LEN 6
#define MACE_MSG_ID_ACK_RXMISSION_MIN_LEN 6
#define MACE_MSG_ID_102_LEN 6
#define MACE_MSG_ID_102_MIN_LEN 6

#define MACE_MSG_ID_ACK_RXMISSION_CRC 244
#define MACE_MSG_ID_102_CRC 244



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ACK_RXMISSION { \
    102, \
    "ACK_RXMISSION", \
    6, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_ack_rxmission_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_ack_rxmission_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_ack_rxmission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_ack_rxmission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_ack_rxmission_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_ack_rxmission_t, mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ACK_RXMISSION { \
    "ACK_RXMISSION", \
    6, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_ack_rxmission_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_ack_rxmission_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_ack_rxmission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_ack_rxmission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_ack_rxmission_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_ack_rxmission_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a ack_rxmission message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The potential new mission state, see MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_ack_rxmission_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ACK_RXMISSION_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_system);
    _mace_put_uint8_t(buf, 2, mission_creator);
    _mace_put_uint8_t(buf, 3, mission_id);
    _mace_put_uint8_t(buf, 4, mission_type);
    _mace_put_uint8_t(buf, 5, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ACK_RXMISSION_LEN);
#else
    mace_ack_rxmission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ACK_RXMISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ACK_RXMISSION;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
}

/**
 * @brief Pack a ack_rxmission message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The potential new mission state, see MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_ack_rxmission_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ACK_RXMISSION_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_system);
    _mace_put_uint8_t(buf, 2, mission_creator);
    _mace_put_uint8_t(buf, 3, mission_id);
    _mace_put_uint8_t(buf, 4, mission_type);
    _mace_put_uint8_t(buf, 5, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ACK_RXMISSION_LEN);
#else
    mace_ack_rxmission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ACK_RXMISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ACK_RXMISSION;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
}

/**
 * @brief Encode a ack_rxmission struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ack_rxmission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_ack_rxmission_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_ack_rxmission_t* ack_rxmission)
{
    return mace_msg_ack_rxmission_pack(system_id, component_id, msg, ack_rxmission->target_system, ack_rxmission->mission_system, ack_rxmission->mission_creator, ack_rxmission->mission_id, ack_rxmission->mission_type, ack_rxmission->mission_state);
}

/**
 * @brief Encode a ack_rxmission struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ack_rxmission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_ack_rxmission_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_ack_rxmission_t* ack_rxmission)
{
    return mace_msg_ack_rxmission_pack_chan(system_id, component_id, chan, msg, ack_rxmission->target_system, ack_rxmission->mission_system, ack_rxmission->mission_creator, ack_rxmission->mission_id, ack_rxmission->mission_type, ack_rxmission->mission_state);
}

/**
 * @brief Send a ack_rxmission message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The potential new mission state, see MISSION_STATE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_ack_rxmission_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ACK_RXMISSION_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_system);
    _mace_put_uint8_t(buf, 2, mission_creator);
    _mace_put_uint8_t(buf, 3, mission_id);
    _mace_put_uint8_t(buf, 4, mission_type);
    _mace_put_uint8_t(buf, 5, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ACK_RXMISSION, buf, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
#else
    mace_ack_rxmission_t packet;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ACK_RXMISSION, (const char *)&packet, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
#endif
}

/**
 * @brief Send a ack_rxmission message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_ack_rxmission_send_struct(mace_channel_t chan, const mace_ack_rxmission_t* ack_rxmission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_ack_rxmission_send(chan, ack_rxmission->target_system, ack_rxmission->mission_system, ack_rxmission->mission_creator, ack_rxmission->mission_id, ack_rxmission->mission_type, ack_rxmission->mission_state);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ACK_RXMISSION, (const char *)ack_rxmission, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
#endif
}

#if MACE_MSG_ID_ACK_RXMISSION_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_ack_rxmission_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_system);
    _mace_put_uint8_t(buf, 2, mission_creator);
    _mace_put_uint8_t(buf, 3, mission_id);
    _mace_put_uint8_t(buf, 4, mission_type);
    _mace_put_uint8_t(buf, 5, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ACK_RXMISSION, buf, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
#else
    mace_ack_rxmission_t *packet = (mace_ack_rxmission_t *)msgbuf;
    packet->target_system = target_system;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ACK_RXMISSION, (const char *)packet, MACE_MSG_ID_ACK_RXMISSION_MIN_LEN, MACE_MSG_ID_ACK_RXMISSION_LEN, MACE_MSG_ID_ACK_RXMISSION_CRC);
#endif
}
#endif

#endif

// MESSAGE ACK_RXMISSION UNPACKING


/**
 * @brief Get field target_system from ack_rxmission message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_ack_rxmission_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_system from ack_rxmission message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_ack_rxmission_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_creator from ack_rxmission message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_ack_rxmission_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_id from ack_rxmission message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_ack_rxmission_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_type from ack_rxmission message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_ack_rxmission_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_state from ack_rxmission message
 *
 * @return The potential new mission state, see MISSION_STATE
 */
static inline uint8_t mace_msg_ack_rxmission_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a ack_rxmission message into a struct
 *
 * @param msg The message to decode
 * @param ack_rxmission C-struct to decode the message contents into
 */
static inline void mace_msg_ack_rxmission_decode(const mace_message_t* msg, mace_ack_rxmission_t* ack_rxmission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    ack_rxmission->target_system = mace_msg_ack_rxmission_get_target_system(msg);
    ack_rxmission->mission_system = mace_msg_ack_rxmission_get_mission_system(msg);
    ack_rxmission->mission_creator = mace_msg_ack_rxmission_get_mission_creator(msg);
    ack_rxmission->mission_id = mace_msg_ack_rxmission_get_mission_id(msg);
    ack_rxmission->mission_type = mace_msg_ack_rxmission_get_mission_type(msg);
    ack_rxmission->mission_state = mace_msg_ack_rxmission_get_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ACK_RXMISSION_LEN? msg->len : MACE_MSG_ID_ACK_RXMISSION_LEN;
        memset(ack_rxmission, 0, MACE_MSG_ID_ACK_RXMISSION_LEN);
    memcpy(ack_rxmission, _MACE_PAYLOAD(msg), len);
#endif
}
