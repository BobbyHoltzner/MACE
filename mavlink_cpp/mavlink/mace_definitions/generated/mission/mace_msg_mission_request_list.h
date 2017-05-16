#pragma once
// MESSAGE MISSION_REQUEST_LIST PACKING

#define MACE_MSG_ID_MISSION_REQUEST_LIST 103

MACEPACKED(
typedef struct __mace_mission_request_list_t {
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
}) mace_mission_request_list_t;

#define MACE_MSG_ID_MISSION_REQUEST_LIST_LEN 4
#define MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN 4
#define MACE_MSG_ID_103_LEN 4
#define MACE_MSG_ID_103_MIN_LEN 4

#define MACE_MSG_ID_MISSION_REQUEST_LIST_CRC 249
#define MACE_MSG_ID_103_CRC 249



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_REQUEST_LIST { \
    103, \
    "MISSION_REQUEST_LIST", \
    4, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_request_list_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_request_list_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_request_list_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_request_list_t, mission_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_REQUEST_LIST { \
    "MISSION_REQUEST_LIST", \
    4, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_request_list_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_request_list_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_request_list_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_request_list_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_list_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN);
#else
    mace_mission_request_list_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_LIST;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
}

/**
 * @brief Pack a mission_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN);
#else
    mace_mission_request_list_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_LIST;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
}

/**
 * @brief Encode a mission_request_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_list_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_request_list_t* mission_request_list)
{
    return mace_msg_mission_request_list_pack(system_id, component_id, msg, mission_request_list->mission_system, mission_request_list->mission_creator, mission_request_list->mission_id, mission_request_list->mission_type);
}

/**
 * @brief Encode a mission_request_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_request_list_t* mission_request_list)
{
    return mace_msg_mission_request_list_pack_chan(system_id, component_id, chan, msg, mission_request_list->mission_system, mission_request_list->mission_creator, mission_request_list->mission_id, mission_request_list->mission_type);
}

/**
 * @brief Send a mission_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_request_list_send(mace_channel_t chan, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_LIST, buf, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
#else
    mace_mission_request_list_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_LIST, (const char *)&packet, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}

/**
 * @brief Send a mission_request_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_request_list_send_struct(mace_channel_t chan, const mace_mission_request_list_t* mission_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_request_list_send(chan, mission_request_list->mission_system, mission_request_list->mission_creator, mission_request_list->mission_id, mission_request_list->mission_type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_LIST, (const char *)mission_request_list, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_REQUEST_LIST_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_request_list_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_LIST, buf, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
#else
    mace_mission_request_list_t *packet = (mace_mission_request_list_t *)msgbuf;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_LIST, (const char *)packet, MACE_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_LIST UNPACKING


/**
 * @brief Get field mission_system from mission_request_list message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_request_list_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_creator from mission_request_list message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_request_list_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_id from mission_request_list message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_request_list_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from mission_request_list message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_request_list_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mission_request_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_list C-struct to decode the message contents into
 */
static inline void mace_msg_mission_request_list_decode(const mace_message_t* msg, mace_mission_request_list_t* mission_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_request_list->mission_system = mace_msg_mission_request_list_get_mission_system(msg);
    mission_request_list->mission_creator = mace_msg_mission_request_list_get_mission_creator(msg);
    mission_request_list->mission_id = mace_msg_mission_request_list_get_mission_id(msg);
    mission_request_list->mission_type = mace_msg_mission_request_list_get_mission_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_REQUEST_LIST_LEN? msg->len : MACE_MSG_ID_MISSION_REQUEST_LIST_LEN;
        memset(mission_request_list, 0, MACE_MSG_ID_MISSION_REQUEST_LIST_LEN);
    memcpy(mission_request_list, _MACE_PAYLOAD(msg), len);
#endif
}
