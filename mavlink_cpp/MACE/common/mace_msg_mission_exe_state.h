#pragma once
// MESSAGE MISSION_EXE_STATE PACKING

#define MACE_MSG_ID_MISSION_EXE_STATE 33

MACEPACKED(
typedef struct __mace_mission_exe_state_t {
 uint8_t mission_state; /*< Defines the current state of the vehicle mission. Useful for determining the next state of the vehicle per mission state.*/
}) mace_mission_exe_state_t;

#define MACE_MSG_ID_MISSION_EXE_STATE_LEN 1
#define MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN 1
#define MACE_MSG_ID_33_LEN 1
#define MACE_MSG_ID_33_MIN_LEN 1

#define MACE_MSG_ID_MISSION_EXE_STATE_CRC 223
#define MACE_MSG_ID_33_CRC 223



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_EXE_STATE { \
    33, \
    "MISSION_EXE_STATE", \
    1, \
    {  { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_exe_state_t, mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_EXE_STATE { \
    "MISSION_EXE_STATE", \
    1, \
    {  { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_exe_state_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_exe_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_state Defines the current state of the vehicle mission. Useful for determining the next state of the vehicle per mission state.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_exe_state_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_EXE_STATE_LEN];
    _mace_put_uint8_t(buf, 0, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_EXE_STATE_LEN);
#else
    mace_mission_exe_state_t packet;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_EXE_STATE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_EXE_STATE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
}

/**
 * @brief Pack a mission_exe_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_state Defines the current state of the vehicle mission. Useful for determining the next state of the vehicle per mission state.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_exe_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_EXE_STATE_LEN];
    _mace_put_uint8_t(buf, 0, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_EXE_STATE_LEN);
#else
    mace_mission_exe_state_t packet;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_EXE_STATE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_EXE_STATE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
}

/**
 * @brief Encode a mission_exe_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_exe_state C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_exe_state_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_exe_state_t* mission_exe_state)
{
    return mace_msg_mission_exe_state_pack(system_id, component_id, msg, mission_exe_state->mission_state);
}

/**
 * @brief Encode a mission_exe_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_exe_state C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_exe_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_exe_state_t* mission_exe_state)
{
    return mace_msg_mission_exe_state_pack_chan(system_id, component_id, chan, msg, mission_exe_state->mission_state);
}

/**
 * @brief Send a mission_exe_state message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_state Defines the current state of the vehicle mission. Useful for determining the next state of the vehicle per mission state.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_exe_state_send(mace_channel_t chan, uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_EXE_STATE_LEN];
    _mace_put_uint8_t(buf, 0, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_EXE_STATE, buf, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
#else
    mace_mission_exe_state_t packet;
    packet.mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_EXE_STATE, (const char *)&packet, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
#endif
}

/**
 * @brief Send a mission_exe_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_exe_state_send_struct(mace_channel_t chan, const mace_mission_exe_state_t* mission_exe_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_exe_state_send(chan, mission_exe_state->mission_state);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_EXE_STATE, (const char *)mission_exe_state, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_EXE_STATE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_exe_state_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_EXE_STATE, buf, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
#else
    mace_mission_exe_state_t *packet = (mace_mission_exe_state_t *)msgbuf;
    packet->mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_EXE_STATE, (const char *)packet, MACE_MSG_ID_MISSION_EXE_STATE_MIN_LEN, MACE_MSG_ID_MISSION_EXE_STATE_LEN, MACE_MSG_ID_MISSION_EXE_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_EXE_STATE UNPACKING


/**
 * @brief Get field mission_state from mission_exe_state message
 *
 * @return Defines the current state of the vehicle mission. Useful for determining the next state of the vehicle per mission state.
 */
static inline uint8_t mace_msg_mission_exe_state_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a mission_exe_state message into a struct
 *
 * @param msg The message to decode
 * @param mission_exe_state C-struct to decode the message contents into
 */
static inline void mace_msg_mission_exe_state_decode(const mace_message_t* msg, mace_mission_exe_state_t* mission_exe_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_exe_state->mission_state = mace_msg_mission_exe_state_get_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_EXE_STATE_LEN? msg->len : MACE_MSG_ID_MISSION_EXE_STATE_LEN;
        memset(mission_exe_state, 0, MACE_MSG_ID_MISSION_EXE_STATE_LEN);
    memcpy(mission_exe_state, _MACE_PAYLOAD(msg), len);
#endif
}
