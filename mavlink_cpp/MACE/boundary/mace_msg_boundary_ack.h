#pragma once
// MESSAGE BOUNDARY_ACK PACKING

#define MACE_MSG_ID_BOUNDARY_ACK 131

MACEPACKED(
typedef struct __mace_boundary_ack_t {
 uint8_t boundary_system; /*< System ID*/
 uint8_t boundary_creator; /*< Creator ID*/
 uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE*/
 uint8_t boundary_result; /*< The acknowledgement result associated, see BOUNDARY_RESULT*/
}) mace_boundary_ack_t;

#define MACE_MSG_ID_BOUNDARY_ACK_LEN 4
#define MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN 4
#define MACE_MSG_ID_131_LEN 4
#define MACE_MSG_ID_131_MIN_LEN 4

#define MACE_MSG_ID_BOUNDARY_ACK_CRC 47
#define MACE_MSG_ID_131_CRC 47



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_BOUNDARY_ACK { \
    131, \
    "BOUNDARY_ACK", \
    4, \
    {  { "boundary_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_boundary_ack_t, boundary_system) }, \
         { "boundary_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_boundary_ack_t, boundary_creator) }, \
         { "boundary_type", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_ack_t, boundary_type) }, \
         { "boundary_result", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_ack_t, boundary_result) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_BOUNDARY_ACK { \
    "BOUNDARY_ACK", \
    4, \
    {  { "boundary_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_boundary_ack_t, boundary_system) }, \
         { "boundary_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_boundary_ack_t, boundary_creator) }, \
         { "boundary_type", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_ack_t, boundary_type) }, \
         { "boundary_result", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_ack_t, boundary_result) }, \
         } \
}
#endif

/**
 * @brief Pack a boundary_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param boundary_result The acknowledgement result associated, see BOUNDARY_RESULT
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_ack_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint8_t boundary_result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_ACK_LEN];
    _mace_put_uint8_t(buf, 0, boundary_system);
    _mace_put_uint8_t(buf, 1, boundary_creator);
    _mace_put_uint8_t(buf, 2, boundary_type);
    _mace_put_uint8_t(buf, 3, boundary_result);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_ACK_LEN);
#else
    mace_boundary_ack_t packet;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;
    packet.boundary_result = boundary_result;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_ACK;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
}

/**
 * @brief Pack a boundary_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param boundary_result The acknowledgement result associated, see BOUNDARY_RESULT
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t boundary_system,uint8_t boundary_creator,uint8_t boundary_type,uint8_t boundary_result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_ACK_LEN];
    _mace_put_uint8_t(buf, 0, boundary_system);
    _mace_put_uint8_t(buf, 1, boundary_creator);
    _mace_put_uint8_t(buf, 2, boundary_type);
    _mace_put_uint8_t(buf, 3, boundary_result);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_ACK_LEN);
#else
    mace_boundary_ack_t packet;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;
    packet.boundary_result = boundary_result;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_ACK;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
}

/**
 * @brief Encode a boundary_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boundary_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_ack_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_boundary_ack_t* boundary_ack)
{
    return mace_msg_boundary_ack_pack(system_id, component_id, msg, boundary_ack->boundary_system, boundary_ack->boundary_creator, boundary_ack->boundary_type, boundary_ack->boundary_result);
}

/**
 * @brief Encode a boundary_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_boundary_ack_t* boundary_ack)
{
    return mace_msg_boundary_ack_pack_chan(system_id, component_id, chan, msg, boundary_ack->boundary_system, boundary_ack->boundary_creator, boundary_ack->boundary_type, boundary_ack->boundary_result);
}

/**
 * @brief Send a boundary_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param boundary_result The acknowledgement result associated, see BOUNDARY_RESULT
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_boundary_ack_send(mace_channel_t chan, uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint8_t boundary_result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_ACK_LEN];
    _mace_put_uint8_t(buf, 0, boundary_system);
    _mace_put_uint8_t(buf, 1, boundary_creator);
    _mace_put_uint8_t(buf, 2, boundary_type);
    _mace_put_uint8_t(buf, 3, boundary_result);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_ACK, buf, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
#else
    mace_boundary_ack_t packet;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;
    packet.boundary_result = boundary_result;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_ACK, (const char *)&packet, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
#endif
}

/**
 * @brief Send a boundary_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_boundary_ack_send_struct(mace_channel_t chan, const mace_boundary_ack_t* boundary_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_boundary_ack_send(chan, boundary_ack->boundary_system, boundary_ack->boundary_creator, boundary_ack->boundary_type, boundary_ack->boundary_result);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_ACK, (const char *)boundary_ack, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
#endif
}

#if MACE_MSG_ID_BOUNDARY_ACK_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_boundary_ack_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint8_t boundary_result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, boundary_system);
    _mace_put_uint8_t(buf, 1, boundary_creator);
    _mace_put_uint8_t(buf, 2, boundary_type);
    _mace_put_uint8_t(buf, 3, boundary_result);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_ACK, buf, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
#else
    mace_boundary_ack_t *packet = (mace_boundary_ack_t *)msgbuf;
    packet->boundary_system = boundary_system;
    packet->boundary_creator = boundary_creator;
    packet->boundary_type = boundary_type;
    packet->boundary_result = boundary_result;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_ACK, (const char *)packet, MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN, MACE_MSG_ID_BOUNDARY_ACK_LEN, MACE_MSG_ID_BOUNDARY_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE BOUNDARY_ACK UNPACKING


/**
 * @brief Get field boundary_system from boundary_ack message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_boundary_ack_get_boundary_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field boundary_creator from boundary_ack message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_boundary_ack_get_boundary_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field boundary_type from boundary_ack message
 *
 * @return Boundary type, see BOUNDARY_TYPE
 */
static inline uint8_t mace_msg_boundary_ack_get_boundary_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field boundary_result from boundary_ack message
 *
 * @return The acknowledgement result associated, see BOUNDARY_RESULT
 */
static inline uint8_t mace_msg_boundary_ack_get_boundary_result(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a boundary_ack message into a struct
 *
 * @param msg The message to decode
 * @param boundary_ack C-struct to decode the message contents into
 */
static inline void mace_msg_boundary_ack_decode(const mace_message_t* msg, mace_boundary_ack_t* boundary_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    boundary_ack->boundary_system = mace_msg_boundary_ack_get_boundary_system(msg);
    boundary_ack->boundary_creator = mace_msg_boundary_ack_get_boundary_creator(msg);
    boundary_ack->boundary_type = mace_msg_boundary_ack_get_boundary_type(msg);
    boundary_ack->boundary_result = mace_msg_boundary_ack_get_boundary_result(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_BOUNDARY_ACK_LEN? msg->len : MACE_MSG_ID_BOUNDARY_ACK_LEN;
        memset(boundary_ack, 0, MACE_MSG_ID_BOUNDARY_ACK_LEN);
    memcpy(boundary_ack, _MACE_PAYLOAD(msg), len);
#endif
}
