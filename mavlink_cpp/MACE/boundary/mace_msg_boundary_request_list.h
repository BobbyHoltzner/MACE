#pragma once
// MESSAGE BOUNDARY_REQUEST_LIST PACKING

#define MACE_MSG_ID_BOUNDARY_REQUEST_LIST 132

MACEPACKED(
typedef struct __mace_boundary_request_list_t {
 uint8_t boundary_host_sysid; /*< sysID of host*/
 uint8_t boundary_host_compid; /*< compid of boundary host*/
 uint8_t boundary_identifier; /*< number on remote instance that identifies the boundary*/
}) mace_boundary_request_list_t;

#define MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN 3
#define MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN 3
#define MACE_MSG_ID_132_LEN 3
#define MACE_MSG_ID_132_MIN_LEN 3

#define MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC 247
#define MACE_MSG_ID_132_CRC 247



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_BOUNDARY_REQUEST_LIST { \
    132, \
    "BOUNDARY_REQUEST_LIST", \
    3, \
    {  { "boundary_host_sysid", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_boundary_request_list_t, boundary_host_sysid) }, \
         { "boundary_host_compid", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_boundary_request_list_t, boundary_host_compid) }, \
         { "boundary_identifier", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_request_list_t, boundary_identifier) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_BOUNDARY_REQUEST_LIST { \
    "BOUNDARY_REQUEST_LIST", \
    3, \
    {  { "boundary_host_sysid", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_boundary_request_list_t, boundary_host_sysid) }, \
         { "boundary_host_compid", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_boundary_request_list_t, boundary_host_compid) }, \
         { "boundary_identifier", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_request_list_t, boundary_identifier) }, \
         } \
}
#endif

/**
 * @brief Pack a boundary_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Boundary type, see boundary_identifier
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_request_list_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, boundary_host_sysid);
    _mace_put_uint8_t(buf, 1, boundary_host_compid);
    _mace_put_uint8_t(buf, 2, boundary_identifier);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN);
#else
    mace_boundary_request_list_t packet;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_REQUEST_LIST;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
}

/**
 * @brief Pack a boundary_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Boundary type, see boundary_identifier
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t boundary_host_sysid,uint8_t boundary_host_compid,uint8_t boundary_identifier)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, boundary_host_sysid);
    _mace_put_uint8_t(buf, 1, boundary_host_compid);
    _mace_put_uint8_t(buf, 2, boundary_identifier);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN);
#else
    mace_boundary_request_list_t packet;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_REQUEST_LIST;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
}

/**
 * @brief Encode a boundary_request_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boundary_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_request_list_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_boundary_request_list_t* boundary_request_list)
{
    return mace_msg_boundary_request_list_pack(system_id, component_id, msg, boundary_request_list->boundary_host_sysid, boundary_request_list->boundary_host_compid, boundary_request_list->boundary_identifier);
}

/**
 * @brief Encode a boundary_request_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_request_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_boundary_request_list_t* boundary_request_list)
{
    return mace_msg_boundary_request_list_pack_chan(system_id, component_id, chan, msg, boundary_request_list->boundary_host_sysid, boundary_request_list->boundary_host_compid, boundary_request_list->boundary_identifier);
}

/**
 * @brief Send a boundary_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Boundary type, see boundary_identifier
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_boundary_request_list_send(mace_channel_t chan, uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, boundary_host_sysid);
    _mace_put_uint8_t(buf, 1, boundary_host_compid);
    _mace_put_uint8_t(buf, 2, boundary_identifier);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST, buf, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
#else
    mace_boundary_request_list_t packet;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST, (const char *)&packet, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
#endif
}

/**
 * @brief Send a boundary_request_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_boundary_request_list_send_struct(mace_channel_t chan, const mace_boundary_request_list_t* boundary_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_boundary_request_list_send(chan, boundary_request_list->boundary_host_sysid, boundary_request_list->boundary_host_compid, boundary_request_list->boundary_identifier);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST, (const char *)boundary_request_list, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
#endif
}

#if MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_boundary_request_list_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, boundary_host_sysid);
    _mace_put_uint8_t(buf, 1, boundary_host_compid);
    _mace_put_uint8_t(buf, 2, boundary_identifier);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST, buf, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
#else
    mace_boundary_request_list_t *packet = (mace_boundary_request_list_t *)msgbuf;
    packet->boundary_host_sysid = boundary_host_sysid;
    packet->boundary_host_compid = boundary_host_compid;
    packet->boundary_identifier = boundary_identifier;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_LIST, (const char *)packet, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE BOUNDARY_REQUEST_LIST UNPACKING


/**
 * @brief Get field boundary_host_sysid from boundary_request_list message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_boundary_request_list_get_boundary_host_sysid(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field boundary_host_compid from boundary_request_list message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_boundary_request_list_get_boundary_host_compid(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field boundary_identifier from boundary_request_list message
 *
 * @return Boundary type, see boundary_identifier
 */
static inline uint8_t mace_msg_boundary_request_list_get_boundary_identifier(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a boundary_request_list message into a struct
 *
 * @param msg The message to decode
 * @param boundary_request_list C-struct to decode the message contents into
 */
static inline void mace_msg_boundary_request_list_decode(const mace_message_t* msg, mace_boundary_request_list_t* boundary_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    boundary_request_list->boundary_host_sysid = mace_msg_boundary_request_list_get_boundary_host_sysid(msg);
    boundary_request_list->boundary_host_compid = mace_msg_boundary_request_list_get_boundary_host_compid(msg);
    boundary_request_list->boundary_identifier = mace_msg_boundary_request_list_get_boundary_identifier(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN? msg->len : MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN;
        memset(boundary_request_list, 0, MACE_MSG_ID_BOUNDARY_REQUEST_LIST_LEN);
    memcpy(boundary_request_list, _MACE_PAYLOAD(msg), len);
#endif
}
