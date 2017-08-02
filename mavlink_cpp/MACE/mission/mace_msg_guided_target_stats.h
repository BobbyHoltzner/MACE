#pragma once
// MESSAGE GUIDED_TARGET_STATS PACKING

#define MACE_MSG_ID_GUIDED_TARGET_STATS 119

MACEPACKED(
typedef struct __mace_guided_target_stats_t {
 float x; /*< X position of this position in the defined coordinate frame*/
 float y; /*< Y position of this position in the defined coordinate frame*/
 float z; /*< Z position of this position in the defined coordinate frame*/
 float distance; /*< Relative distance away the system is from the target location.*/
 uint8_t coordinate_frame; /*< Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition.*/
}) mace_guided_target_stats_t;

#define MACE_MSG_ID_GUIDED_TARGET_STATS_LEN 17
#define MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN 17
#define MACE_MSG_ID_119_LEN 17
#define MACE_MSG_ID_119_MIN_LEN 17

#define MACE_MSG_ID_GUIDED_TARGET_STATS_CRC 109
#define MACE_MSG_ID_119_CRC 109



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GUIDED_TARGET_STATS { \
    119, \
    "GUIDED_TARGET_STATS", \
    5, \
    {  { "x", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_guided_target_stats_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_guided_target_stats_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_guided_target_stats_t, z) }, \
         { "distance", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_guided_target_stats_t, distance) }, \
         { "coordinate_frame", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_guided_target_stats_t, coordinate_frame) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GUIDED_TARGET_STATS { \
    "GUIDED_TARGET_STATS", \
    5, \
    {  { "x", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_guided_target_stats_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_guided_target_stats_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_guided_target_stats_t, z) }, \
         { "distance", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_guided_target_stats_t, distance) }, \
         { "coordinate_frame", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_guided_target_stats_t, coordinate_frame) }, \
         } \
}
#endif

/**
 * @brief Pack a guided_target_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param coordinate_frame Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition.
 * @param x X position of this position in the defined coordinate frame
 * @param y Y position of this position in the defined coordinate frame
 * @param z Z position of this position in the defined coordinate frame
 * @param distance Relative distance away the system is from the target location.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_guided_target_stats_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t coordinate_frame, float x, float y, float z, float distance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GUIDED_TARGET_STATS_LEN];
    _mace_put_float(buf, 0, x);
    _mace_put_float(buf, 4, y);
    _mace_put_float(buf, 8, z);
    _mace_put_float(buf, 12, distance);
    _mace_put_uint8_t(buf, 16, coordinate_frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN);
#else
    mace_guided_target_stats_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.distance = distance;
    packet.coordinate_frame = coordinate_frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GUIDED_TARGET_STATS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
}

/**
 * @brief Pack a guided_target_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param coordinate_frame Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition.
 * @param x X position of this position in the defined coordinate frame
 * @param y Y position of this position in the defined coordinate frame
 * @param z Z position of this position in the defined coordinate frame
 * @param distance Relative distance away the system is from the target location.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_guided_target_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t coordinate_frame,float x,float y,float z,float distance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GUIDED_TARGET_STATS_LEN];
    _mace_put_float(buf, 0, x);
    _mace_put_float(buf, 4, y);
    _mace_put_float(buf, 8, z);
    _mace_put_float(buf, 12, distance);
    _mace_put_uint8_t(buf, 16, coordinate_frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN);
#else
    mace_guided_target_stats_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.distance = distance;
    packet.coordinate_frame = coordinate_frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GUIDED_TARGET_STATS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
}

/**
 * @brief Encode a guided_target_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param guided_target_stats C-struct to read the message contents from
 */
static inline uint16_t mace_msg_guided_target_stats_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_guided_target_stats_t* guided_target_stats)
{
    return mace_msg_guided_target_stats_pack(system_id, component_id, msg, guided_target_stats->coordinate_frame, guided_target_stats->x, guided_target_stats->y, guided_target_stats->z, guided_target_stats->distance);
}

/**
 * @brief Encode a guided_target_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param guided_target_stats C-struct to read the message contents from
 */
static inline uint16_t mace_msg_guided_target_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_guided_target_stats_t* guided_target_stats)
{
    return mace_msg_guided_target_stats_pack_chan(system_id, component_id, chan, msg, guided_target_stats->coordinate_frame, guided_target_stats->x, guided_target_stats->y, guided_target_stats->z, guided_target_stats->distance);
}

/**
 * @brief Send a guided_target_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param coordinate_frame Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition.
 * @param x X position of this position in the defined coordinate frame
 * @param y Y position of this position in the defined coordinate frame
 * @param z Z position of this position in the defined coordinate frame
 * @param distance Relative distance away the system is from the target location.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_guided_target_stats_send(mace_channel_t chan, uint8_t coordinate_frame, float x, float y, float z, float distance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GUIDED_TARGET_STATS_LEN];
    _mace_put_float(buf, 0, x);
    _mace_put_float(buf, 4, y);
    _mace_put_float(buf, 8, z);
    _mace_put_float(buf, 12, distance);
    _mace_put_uint8_t(buf, 16, coordinate_frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GUIDED_TARGET_STATS, buf, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
#else
    mace_guided_target_stats_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.distance = distance;
    packet.coordinate_frame = coordinate_frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GUIDED_TARGET_STATS, (const char *)&packet, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
#endif
}

/**
 * @brief Send a guided_target_stats message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_guided_target_stats_send_struct(mace_channel_t chan, const mace_guided_target_stats_t* guided_target_stats)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_guided_target_stats_send(chan, guided_target_stats->coordinate_frame, guided_target_stats->x, guided_target_stats->y, guided_target_stats->z, guided_target_stats->distance);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GUIDED_TARGET_STATS, (const char *)guided_target_stats, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
#endif
}

#if MACE_MSG_ID_GUIDED_TARGET_STATS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_guided_target_stats_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t coordinate_frame, float x, float y, float z, float distance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, x);
    _mace_put_float(buf, 4, y);
    _mace_put_float(buf, 8, z);
    _mace_put_float(buf, 12, distance);
    _mace_put_uint8_t(buf, 16, coordinate_frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GUIDED_TARGET_STATS, buf, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
#else
    mace_guided_target_stats_t *packet = (mace_guided_target_stats_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->distance = distance;
    packet->coordinate_frame = coordinate_frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GUIDED_TARGET_STATS, (const char *)packet, MACE_MSG_ID_GUIDED_TARGET_STATS_MIN_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN, MACE_MSG_ID_GUIDED_TARGET_STATS_CRC);
#endif
}
#endif

#endif

// MESSAGE GUIDED_TARGET_STATS UNPACKING


/**
 * @brief Get field coordinate_frame from guided_target_stats message
 *
 * @return Coordinate frame of the position vector. This field is as related to the MAV_FRAME definition.
 */
static inline uint8_t mace_msg_guided_target_stats_get_coordinate_frame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field x from guided_target_stats message
 *
 * @return X position of this position in the defined coordinate frame
 */
static inline float mace_msg_guided_target_stats_get_x(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from guided_target_stats message
 *
 * @return Y position of this position in the defined coordinate frame
 */
static inline float mace_msg_guided_target_stats_get_y(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from guided_target_stats message
 *
 * @return Z position of this position in the defined coordinate frame
 */
static inline float mace_msg_guided_target_stats_get_z(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field distance from guided_target_stats message
 *
 * @return Relative distance away the system is from the target location.
 */
static inline float mace_msg_guided_target_stats_get_distance(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a guided_target_stats message into a struct
 *
 * @param msg The message to decode
 * @param guided_target_stats C-struct to decode the message contents into
 */
static inline void mace_msg_guided_target_stats_decode(const mace_message_t* msg, mace_guided_target_stats_t* guided_target_stats)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    guided_target_stats->x = mace_msg_guided_target_stats_get_x(msg);
    guided_target_stats->y = mace_msg_guided_target_stats_get_y(msg);
    guided_target_stats->z = mace_msg_guided_target_stats_get_z(msg);
    guided_target_stats->distance = mace_msg_guided_target_stats_get_distance(msg);
    guided_target_stats->coordinate_frame = mace_msg_guided_target_stats_get_coordinate_frame(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GUIDED_TARGET_STATS_LEN? msg->len : MACE_MSG_ID_GUIDED_TARGET_STATS_LEN;
        memset(guided_target_stats, 0, MACE_MSG_ID_GUIDED_TARGET_STATS_LEN);
    memcpy(guided_target_stats, _MACE_PAYLOAD(msg), len);
#endif
}
