#pragma once
// MESSAGE BOUNDARY_COUNT PACKING

#define MACE_MSG_ID_BOUNDARY_COUNT 133

MACEPACKED(
typedef struct __mace_boundary_count_t {
 uint16_t count; /*< Number of items defining the boundary.*/
 uint8_t boundary_system; /*< System ID*/
 uint8_t boundary_creator; /*< Creator ID*/
 uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE*/
}) mace_boundary_count_t;

#define MACE_MSG_ID_BOUNDARY_COUNT_LEN 5
#define MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN 5
#define MACE_MSG_ID_133_LEN 5
#define MACE_MSG_ID_133_MIN_LEN 5

#define MACE_MSG_ID_BOUNDARY_COUNT_CRC 168
#define MACE_MSG_ID_133_CRC 168



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_BOUNDARY_COUNT { \
    133, \
    "BOUNDARY_COUNT", \
    4, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_boundary_count_t, count) }, \
         { "boundary_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_count_t, boundary_system) }, \
         { "boundary_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_count_t, boundary_creator) }, \
         { "boundary_type", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_boundary_count_t, boundary_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_BOUNDARY_COUNT { \
    "BOUNDARY_COUNT", \
    4, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_boundary_count_t, count) }, \
         { "boundary_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_count_t, boundary_system) }, \
         { "boundary_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_count_t, boundary_creator) }, \
         { "boundary_type", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_boundary_count_t, boundary_type) }, \
         } \
}
#endif

/**
 * @brief Pack a boundary_count message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param count Number of items defining the boundary.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_count_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, boundary_system);
    _mace_put_uint8_t(buf, 3, boundary_creator);
    _mace_put_uint8_t(buf, 4, boundary_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_COUNT_LEN);
#else
    mace_boundary_count_t packet;
    packet.count = count;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_COUNT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_COUNT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
}

/**
 * @brief Pack a boundary_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param count Number of items defining the boundary.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t boundary_system,uint8_t boundary_creator,uint8_t boundary_type,uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, boundary_system);
    _mace_put_uint8_t(buf, 3, boundary_creator);
    _mace_put_uint8_t(buf, 4, boundary_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_COUNT_LEN);
#else
    mace_boundary_count_t packet;
    packet.count = count;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_COUNT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_COUNT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
}

/**
 * @brief Encode a boundary_count struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boundary_count C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_count_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_boundary_count_t* boundary_count)
{
    return mace_msg_boundary_count_pack(system_id, component_id, msg, boundary_count->boundary_system, boundary_count->boundary_creator, boundary_count->boundary_type, boundary_count->count);
}

/**
 * @brief Encode a boundary_count struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_count C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_count_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_boundary_count_t* boundary_count)
{
    return mace_msg_boundary_count_pack_chan(system_id, component_id, chan, msg, boundary_count->boundary_system, boundary_count->boundary_creator, boundary_count->boundary_type, boundary_count->count);
}

/**
 * @brief Send a boundary_count message
 * @param chan MAVLink channel to send the message
 *
 * @param boundary_system System ID
 * @param boundary_creator Creator ID
 * @param boundary_type Boundary type, see BOUNDARY_TYPE
 * @param count Number of items defining the boundary.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_boundary_count_send(mace_channel_t chan, uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, boundary_system);
    _mace_put_uint8_t(buf, 3, boundary_creator);
    _mace_put_uint8_t(buf, 4, boundary_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_COUNT, buf, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
#else
    mace_boundary_count_t packet;
    packet.count = count;
    packet.boundary_system = boundary_system;
    packet.boundary_creator = boundary_creator;
    packet.boundary_type = boundary_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_COUNT, (const char *)&packet, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
#endif
}

/**
 * @brief Send a boundary_count message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_boundary_count_send_struct(mace_channel_t chan, const mace_boundary_count_t* boundary_count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_boundary_count_send(chan, boundary_count->boundary_system, boundary_count->boundary_creator, boundary_count->boundary_type, boundary_count->count);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_COUNT, (const char *)boundary_count, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
#endif
}

#if MACE_MSG_ID_BOUNDARY_COUNT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_boundary_count_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t boundary_system, uint8_t boundary_creator, uint8_t boundary_type, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, boundary_system);
    _mace_put_uint8_t(buf, 3, boundary_creator);
    _mace_put_uint8_t(buf, 4, boundary_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_COUNT, buf, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
#else
    mace_boundary_count_t *packet = (mace_boundary_count_t *)msgbuf;
    packet->count = count;
    packet->boundary_system = boundary_system;
    packet->boundary_creator = boundary_creator;
    packet->boundary_type = boundary_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_COUNT, (const char *)packet, MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN, MACE_MSG_ID_BOUNDARY_COUNT_LEN, MACE_MSG_ID_BOUNDARY_COUNT_CRC);
#endif
}
#endif

#endif

// MESSAGE BOUNDARY_COUNT UNPACKING


/**
 * @brief Get field boundary_system from boundary_count message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_boundary_count_get_boundary_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field boundary_creator from boundary_count message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_boundary_count_get_boundary_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field boundary_type from boundary_count message
 *
 * @return Boundary type, see BOUNDARY_TYPE
 */
static inline uint8_t mace_msg_boundary_count_get_boundary_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field count from boundary_count message
 *
 * @return Number of items defining the boundary.
 */
static inline uint16_t mace_msg_boundary_count_get_count(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a boundary_count message into a struct
 *
 * @param msg The message to decode
 * @param boundary_count C-struct to decode the message contents into
 */
static inline void mace_msg_boundary_count_decode(const mace_message_t* msg, mace_boundary_count_t* boundary_count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    boundary_count->count = mace_msg_boundary_count_get_count(msg);
    boundary_count->boundary_system = mace_msg_boundary_count_get_boundary_system(msg);
    boundary_count->boundary_creator = mace_msg_boundary_count_get_boundary_creator(msg);
    boundary_count->boundary_type = mace_msg_boundary_count_get_boundary_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_BOUNDARY_COUNT_LEN? msg->len : MACE_MSG_ID_BOUNDARY_COUNT_LEN;
        memset(boundary_count, 0, MACE_MSG_ID_BOUNDARY_COUNT_LEN);
    memcpy(boundary_count, _MACE_PAYLOAD(msg), len);
#endif
}