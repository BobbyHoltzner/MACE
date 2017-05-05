#pragma once
// MESSAGE MACE_ROI_AG PACKING

#define MAVLINK_MSG_ID_MACE_ROI_AG 195

MAVPACKED(
typedef struct __mavlink_mace_roi_ag_t {
 float stress_value; /*< Stress value*/
 float x; /*< PARAM5 / local: x position, global: latitude*/
 float y; /*< PARAM6 / y position: global: longitude*/
 float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t point_discovery; /*< See POINT_DISCOVERY enum*/
 uint8_t stress_threshold; /*< See STRESS_THRESHOLD enum*/
 uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h*/
}) mavlink_mace_roi_ag_t;

#define MAVLINK_MSG_ID_MACE_ROI_AG_LEN 21
#define MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN 21
#define MAVLINK_MSG_ID_195_LEN 21
#define MAVLINK_MSG_ID_195_MIN_LEN 21

#define MAVLINK_MSG_ID_MACE_ROI_AG_CRC 197
#define MAVLINK_MSG_ID_195_CRC 197



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MACE_ROI_AG { \
    195, \
    "MACE_ROI_AG", \
    9, \
    {  { "stress_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mace_roi_ag_t, stress_value) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mace_roi_ag_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mace_roi_ag_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mace_roi_ag_t, z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mace_roi_ag_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_mace_roi_ag_t, target_component) }, \
         { "point_discovery", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_mace_roi_ag_t, point_discovery) }, \
         { "stress_threshold", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_mace_roi_ag_t, stress_threshold) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mace_roi_ag_t, frame) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MACE_ROI_AG { \
    "MACE_ROI_AG", \
    9, \
    {  { "stress_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mace_roi_ag_t, stress_value) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mace_roi_ag_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mace_roi_ag_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mace_roi_ag_t, z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mace_roi_ag_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_mace_roi_ag_t, target_component) }, \
         { "point_discovery", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_mace_roi_ag_t, point_discovery) }, \
         { "stress_threshold", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_mace_roi_ag_t, stress_threshold) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mace_roi_ag_t, frame) }, \
         } \
}
#endif

/**
 * @brief Pack a mace_roi_ag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_roi_ag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ROI_AG_LEN];
    _mav_put_float(buf, 0, stress_value);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, point_discovery);
    _mav_put_uint8_t(buf, 19, stress_threshold);
    _mav_put_uint8_t(buf, 20, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_ROI_AG_LEN);
#else
    mavlink_mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_ROI_AG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_ROI_AG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
}

/**
 * @brief Pack a mace_roi_ag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mace_roi_ag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t point_discovery,uint8_t stress_threshold,float stress_value,uint8_t frame,float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ROI_AG_LEN];
    _mav_put_float(buf, 0, stress_value);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, point_discovery);
    _mav_put_uint8_t(buf, 19, stress_threshold);
    _mav_put_uint8_t(buf, 20, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MACE_ROI_AG_LEN);
#else
    mavlink_mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MACE_ROI_AG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MACE_ROI_AG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
}

/**
 * @brief Encode a mace_roi_ag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mace_roi_ag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_roi_ag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mace_roi_ag_t* mace_roi_ag)
{
    return mavlink_msg_mace_roi_ag_pack(system_id, component_id, msg, mace_roi_ag->target_system, mace_roi_ag->target_component, mace_roi_ag->point_discovery, mace_roi_ag->stress_threshold, mace_roi_ag->stress_value, mace_roi_ag->frame, mace_roi_ag->x, mace_roi_ag->y, mace_roi_ag->z);
}

/**
 * @brief Encode a mace_roi_ag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mace_roi_ag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mace_roi_ag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mace_roi_ag_t* mace_roi_ag)
{
    return mavlink_msg_mace_roi_ag_pack_chan(system_id, component_id, chan, msg, mace_roi_ag->target_system, mace_roi_ag->target_component, mace_roi_ag->point_discovery, mace_roi_ag->stress_threshold, mace_roi_ag->stress_value, mace_roi_ag->frame, mace_roi_ag->x, mace_roi_ag->y, mace_roi_ag->z);
}

/**
 * @brief Send a mace_roi_ag message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mace_roi_ag_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MACE_ROI_AG_LEN];
    _mav_put_float(buf, 0, stress_value);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, point_discovery);
    _mav_put_uint8_t(buf, 19, stress_threshold);
    _mav_put_uint8_t(buf, 20, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ROI_AG, buf, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
#else
    mavlink_mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ROI_AG, (const char *)&packet, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
#endif
}

/**
 * @brief Send a mace_roi_ag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mace_roi_ag_send_struct(mavlink_channel_t chan, const mavlink_mace_roi_ag_t* mace_roi_ag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mace_roi_ag_send(chan, mace_roi_ag->target_system, mace_roi_ag->target_component, mace_roi_ag->point_discovery, mace_roi_ag->stress_threshold, mace_roi_ag->stress_value, mace_roi_ag->frame, mace_roi_ag->x, mace_roi_ag->y, mace_roi_ag->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ROI_AG, (const char *)mace_roi_ag, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
#endif
}

#if MAVLINK_MSG_ID_MACE_ROI_AG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mace_roi_ag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, stress_value);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, point_discovery);
    _mav_put_uint8_t(buf, 19, stress_threshold);
    _mav_put_uint8_t(buf, 20, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ROI_AG, buf, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
#else
    mavlink_mace_roi_ag_t *packet = (mavlink_mace_roi_ag_t *)msgbuf;
    packet->stress_value = stress_value;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->point_discovery = point_discovery;
    packet->stress_threshold = stress_threshold;
    packet->frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MACE_ROI_AG, (const char *)packet, MAVLINK_MSG_ID_MACE_ROI_AG_MIN_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_LEN, MAVLINK_MSG_ID_MACE_ROI_AG_CRC);
#endif
}
#endif

#endif

// MESSAGE MACE_ROI_AG UNPACKING


/**
 * @brief Get field target_system from mace_roi_ag message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mace_roi_ag_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from mace_roi_ag message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mace_roi_ag_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field point_discovery from mace_roi_ag message
 *
 * @return See POINT_DISCOVERY enum
 */
static inline uint8_t mavlink_msg_mace_roi_ag_get_point_discovery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field stress_threshold from mace_roi_ag message
 *
 * @return See STRESS_THRESHOLD enum
 */
static inline uint8_t mavlink_msg_mace_roi_ag_get_stress_threshold(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field stress_value from mace_roi_ag message
 *
 * @return Stress value
 */
static inline float mavlink_msg_mace_roi_ag_get_stress_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field frame from mace_roi_ag message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mavlink_msg_mace_roi_ag_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field x from mace_roi_ag message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline float mavlink_msg_mace_roi_ag_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from mace_roi_ag message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline float mavlink_msg_mace_roi_ag_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from mace_roi_ag message
 *
 * @return PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
static inline float mavlink_msg_mace_roi_ag_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a mace_roi_ag message into a struct
 *
 * @param msg The message to decode
 * @param mace_roi_ag C-struct to decode the message contents into
 */
static inline void mavlink_msg_mace_roi_ag_decode(const mavlink_message_t* msg, mavlink_mace_roi_ag_t* mace_roi_ag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mace_roi_ag->stress_value = mavlink_msg_mace_roi_ag_get_stress_value(msg);
    mace_roi_ag->x = mavlink_msg_mace_roi_ag_get_x(msg);
    mace_roi_ag->y = mavlink_msg_mace_roi_ag_get_y(msg);
    mace_roi_ag->z = mavlink_msg_mace_roi_ag_get_z(msg);
    mace_roi_ag->target_system = mavlink_msg_mace_roi_ag_get_target_system(msg);
    mace_roi_ag->target_component = mavlink_msg_mace_roi_ag_get_target_component(msg);
    mace_roi_ag->point_discovery = mavlink_msg_mace_roi_ag_get_point_discovery(msg);
    mace_roi_ag->stress_threshold = mavlink_msg_mace_roi_ag_get_stress_threshold(msg);
    mace_roi_ag->frame = mavlink_msg_mace_roi_ag_get_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MACE_ROI_AG_LEN? msg->len : MAVLINK_MSG_ID_MACE_ROI_AG_LEN;
        memset(mace_roi_ag, 0, MAVLINK_MSG_ID_MACE_ROI_AG_LEN);
    memcpy(mace_roi_ag, _MAV_PAYLOAD(msg), len);
#endif
}
