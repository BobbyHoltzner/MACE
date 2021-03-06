/** @file
 *  @brief MAVLink comm protocol generated from mission.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_MISSION_H
#define MACE_MISSION_H

#ifndef MACE_H
    #error Wrong include order: MACE_MISSION.H MUST NOT BE DIRECTLY USED. Include mace.h from the same directory instead or set ALL AND EVERY defines from MACE.H manually accordingly, including the #define MACE_H call.
#endif

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 2

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MACE_MESSAGE_LENGTHS
#define MACE_MESSAGE_LENGTHS {}
#endif

#ifndef MACE_MESSAGE_CRCS
#define MACE_MESSAGE_CRCS {{100, 132, 5, 0, 0, 0}, {101, 18, 7, 1, 2, 0}, {102, 87, 7, 0, 0, 0}, {103, 135, 3, 0, 0, 0}, {104, 84, 5, 0, 0, 0}, {105, 165, 8, 1, 2, 0}, {106, 3, 8, 1, 2, 0}, {107, 49, 41, 1, 32, 0}, {108, 4, 7, 3, 4, 5}, {109, 168, 7, 3, 4, 5}, {110, 224, 4, 0, 0, 0}, {111, 156, 5, 0, 0, 0}, {112, 47, 7, 0, 0, 0}, {113, 54, 7, 0, 0, 0}, {114, 34, 4, 1, 0, 0}, {115, 55, 5, 0, 0, 0}, {116, 36, 1, 1, 0, 0}, {117, 149, 53, 0, 0, 0}, {118, 85, 53, 1, 52, 0}, {119, 21, 2, 1, 0, 0}, {120, 109, 17, 0, 0, 0}}
#endif

#include "../mace_protocol.h"

#define MACE_ENABLED_MISSION

// ENUM DEFINITIONS


/** @brief  The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI). */
#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI
{
   MAV_ROI_NONE=0, /* No region of interest. | */
   MAV_ROI_WPNEXT=1, /* Point toward next MISSION. | */
   MAV_ROI_WPINDEX=2, /* Point toward given MISSION. | */
   MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
   MAV_ROI_TARGET=4, /* Point toward of given id. | */
   MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

/** @brief result in a mavlink mission ack */
#ifndef HAVE_ENUM_MAV_MISSION_RESULT
#define HAVE_ENUM_MAV_MISSION_RESULT
typedef enum MAV_MISSION_RESULT
{
   MAV_MISSION_ACCEPTED=0, /* mission accepted OK | */
   MAV_MISSION_ERROR=1, /* generic error / not accepting mission commands at all right now | */
   MAV_MISSION_UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
   MAV_MISSION_UNSUPPORTED=3, /* command is not supported | */
   MAV_MISSION_NO_SPACE=4, /* mission item exceeds storage space | */
   MAV_MISSION_INVALID=5, /* one of the parameters has an invalid value | */
   MAV_MISSION_INVALID_PARAM1=6, /* param1 has an invalid value | */
   MAV_MISSION_INVALID_PARAM2=7, /* param2 has an invalid value | */
   MAV_MISSION_INVALID_PARAM3=8, /* param3 has an invalid value | */
   MAV_MISSION_INVALID_PARAM4=9, /* param4 has an invalid value | */
   MAV_MISSION_INVALID_PARAM5_X=10, /* x/param5 has an invalid value | */
   MAV_MISSION_INVALID_PARAM6_Y=11, /* y/param6 has an invalid value | */
   MAV_MISSION_INVALID_PARAM7=12, /* param7 has an invalid value | */
   MAV_MISSION_INVALID_SEQUENCE=13, /* received waypoint out of sequence | */
   MAV_MISSION_DENIED=14, /* not accepting any mission commands from this communication partner | */
   MAV_MISSION_DOES_NOT_EXIST=15, /* the requested mission with the associated key does not exist. | */
   MAV_MISSION_RESULT_ENUM_END=16, /*  | */
} MAV_MISSION_RESULT;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MAV_MISSION_TYPE
#define HAVE_ENUM_MAV_MISSION_TYPE
typedef enum MAV_MISSION_TYPE
{
   MAV_MISSION_TYPE_AUTO=0, /* Items are mission commands for main auto mission. | */
   MAV_MISSION_TYPE_GUIDED=1, /* Items are mission commands for guided mission. | */
   MAV_MISSION_TYPE_ROI=2, /* Items are regions of interest that the vehicle should visit in a guided mission. | */
   MAV_MISSION_TYPE_FENCE=3, /* Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items. | */
   MAV_MISSION_TYPE_RALLY=4, /* Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT rally point items. | */
   MAV_MISSION_TYPE_ALL=255, /* Only used in MISSION_CLEAR_ALL to clear all mission types. | */
   MAV_MISSION_TYPE_ENUM_END=256, /*  | */
} MAV_MISSION_TYPE;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MAV_MISSION_STATE
#define HAVE_ENUM_MAV_MISSION_STATE
typedef enum MAV_MISSION_STATE
{
   MAV_MISSION_CURRENT=0, /* Items are mission commands for main auto mission and are received via the autopilot and aircraft module. | */
   MAV_MISSION_ONBOARD=1, /* Items are mission ready and have been acknowledged by the aircraft module for processing. | */
   MAV_MISSION_PROPOSED=2, /* Items are mission ready but have been generated by a module not related to the aircraft and need to be pushed towards the appropriate aircraft module. | */
   MAV_MISSION_RECEIVED=3, /* Items have been distributed and received by the appropriate MACE instance connected to the associated module. They however have yet to be marked as current or onboard. Often this state is reflected via communication between external link modules. | */
   MAV_MISSION_OUTDATED=4, /* Items were once relevant to the system have been since replaced with newer current missions. | */
   MAV_MISSION_STATE_ENUM_END=5, /*  | */
} MAV_MISSION_STATE;
#endif

// MACE VERSION

#ifndef MACE_VERSION
#define MACE_VERSION 3
#endif

#if (MACE_VERSION == 0)
#undef MACE_VERSION
#define MACE_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mace_msg_new_onboard_mission.h"
#include "./mace_msg_new_proposed_mission.h"
#include "./mace_msg_mission_ack.h"
#include "./mace_msg_mission_request_list_generic.h"
#include "./mace_msg_mission_request_list.h"
#include "./mace_msg_mission_count.h"
#include "./mace_msg_mission_request_item.h"
#include "./mace_msg_mission_item.h"
#include "./mace_msg_mission_request_partial_list.h"
#include "./mace_msg_mission_write_partial_list.h"
#include "./mace_msg_starting_current_mission.h"
#include "./mace_msg_mission_set_current.h"
#include "./mace_msg_mission_item_current.h"
#include "./mace_msg_mission_item_reached.h"
#include "./mace_msg_mission_clear.h"
#include "./mace_msg_mission_exe_state.h"
#include "./mace_msg_mission_request_home.h"
#include "./mace_msg_home_position.h"
#include "./mace_msg_set_home_position.h"
#include "./mace_msg_home_position_ack.h"
#include "./mace_msg_guided_target_stats.h"

// base include
#include "../common/common.h"

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 2

#if MACE_THIS_XML_IDX == MACE_PRIMARY_XML_IDX
# define MACE_MESSAGE_INFO {MACE_MESSAGE_INFO_NEW_ONBOARD_MISSION, MACE_MESSAGE_INFO_NEW_PROPOSED_MISSION, MACE_MESSAGE_INFO_MISSION_ACK, MACE_MESSAGE_INFO_MISSION_REQUEST_LIST_GENERIC, MACE_MESSAGE_INFO_MISSION_REQUEST_LIST, MACE_MESSAGE_INFO_MISSION_COUNT, MACE_MESSAGE_INFO_MISSION_REQUEST_ITEM, MACE_MESSAGE_INFO_MISSION_ITEM, MACE_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST, MACE_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST, MACE_MESSAGE_INFO_STARTING_CURRENT_MISSION, MACE_MESSAGE_INFO_MISSION_SET_CURRENT, MACE_MESSAGE_INFO_MISSION_ITEM_CURRENT, MACE_MESSAGE_INFO_MISSION_ITEM_REACHED, MACE_MESSAGE_INFO_MISSION_CLEAR, MACE_MESSAGE_INFO_MISSION_EXE_STATE, MACE_MESSAGE_INFO_MISSION_REQUEST_HOME, MACE_MESSAGE_INFO_HOME_POSITION, MACE_MESSAGE_INFO_SET_HOME_POSITION, MACE_MESSAGE_INFO_HOME_POSITION_ACK, MACE_MESSAGE_INFO_GUIDED_TARGET_STATS}
# if MACE_COMMAND_24BIT
#  include "../mace_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_MISSION_H
