/** @file
 *  @brief MAVLink comm protocol generated from MACE.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_MACE_H
#define MAVLINK_MACE_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_MACE.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{150, 134, 42, 0, 0, 0}, {151, 219, 8, 3, 6, 7}, {152, 208, 4, 0, 0, 0}, {153, 188, 12, 0, 0, 0}, {154, 84, 15, 3, 6, 7}, {155, 22, 13, 3, 4, 5}, {156, 19, 6, 3, 0, 1}, {157, 21, 15, 3, 12, 13}, {158, 134, 14, 3, 12, 13}, {160, 78, 12, 3, 8, 9}, {161, 68, 3, 3, 0, 1}, {162, 189, 8, 0, 0, 0}, {163, 127, 28, 0, 0, 0}, {164, 154, 44, 0, 0, 0}, {165, 21, 3, 0, 0, 0}, {166, 21, 9, 0, 0, 0}, {167, 144, 22, 0, 0, 0}, {168, 1, 12, 0, 0, 0}, {169, 234, 18, 0, 0, 0}, {170, 73, 34, 0, 0, 0}, {171, 181, 66, 0, 0, 0}, {172, 22, 98, 0, 0, 0}, {173, 83, 8, 0, 0, 0}, {174, 167, 48, 0, 0, 0}, {175, 138, 19, 3, 14, 15}, {176, 234, 3, 3, 0, 1}, {177, 240, 20, 0, 0, 0}, {178, 47, 24, 0, 0, 0}, {179, 189, 29, 1, 26, 0}, {180, 52, 45, 1, 42, 0}, {181, 174, 4, 0, 0, 0}, {182, 229, 40, 0, 0, 0}, {183, 85, 2, 3, 0, 1}, {184, 159, 206, 3, 4, 5}, {185, 186, 7, 3, 4, 5}, {186, 72, 29, 3, 0, 1}, {191, 92, 27, 0, 0, 0}, {192, 36, 44, 0, 0, 0}, {193, 71, 22, 0, 0, 0}, {194, 98, 25, 0, 0, 0}, {200, 134, 42, 3, 40, 41}, {201, 205, 14, 3, 12, 13}, {214, 69, 8, 3, 6, 7}, {215, 101, 3, 0, 0, 0}, {216, 50, 3, 3, 0, 1}, {217, 202, 6, 0, 0, 0}, {218, 17, 7, 3, 0, 1}, {219, 162, 2, 0, 0, 0}, {226, 207, 8, 0, 0, 0}, {300, 115, 7, 3, 4, 5}, {301, 182, 7, 3, 4, 5}, {302, 18, 38, 3, 32, 33}, {303, 255, 5, 3, 2, 3}, {304, 83, 5, 3, 2, 3}, {305, 88, 2, 0, 0, 0}, {306, 193, 3, 3, 0, 1}, {307, 122, 5, 3, 2, 3}, {308, 93, 3, 3, 0, 1}, {309, 190, 3, 0, 0, 0}, {310, 22, 4, 3, 0, 1}, {311, 163, 8, 3, 4, 5}, {11000, 134, 51, 3, 4, 5}, {11001, 15, 135, 0, 0, 0}, {11002, 234, 179, 3, 4, 5}, {11003, 64, 5, 0, 0, 0}, {11010, 46, 49, 0, 0, 0}, {11011, 106, 44, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_MACE

// ENUM DEFINITIONS


/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MACE_MISSION_TYPE
#define HAVE_ENUM_MACE_MISSION_TYPE
typedef enum MACE_MISSION_TYPE
{
   MACE_MISSION_TYPE_AUTO=0, /* Items are mission commands for main auto mission. | */
   MACE_MISSION_TYPE_AUTO_PROPOSED=1, /* Items are mission commands for main auto proposed mission. | */
   MACE_MISSION_TYPE_GUIDED=2, /* Items are mission commands for guided mission. | */
   MACE_MISSION_TYPE_GUIDED_PROPOSED=3, /* Items are mission commands for guided proposed mission. | */
   MACE_MISSION_TYPE_ROI=4, /* Items are regions of interest that the vehicle should visit in a guided mission. | */
   MACE_MISSION_TYPE_FENCE=5, /* Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items. | */
   MACE_MISSION_TYPE_RALLY=6, /* Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT rally point items. | */
   MACE_MISSION_TYPE_ALL=255, /* Only used in MISSION_CLEAR_ALL to clear all mission types. | */
   MACE_MISSION_TYPE_ENUM_END=256, /*  | */
} MACE_MISSION_TYPE;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MACE_POINT_DISCOVERY
#define HAVE_ENUM_MACE_POINT_DISCOVERY
typedef enum MACE_POINT_DISCOVERY
{
   MACE_POINT_DISCOVERY_EXISTING=0, /* The point reported is an existing point in the list relative to the original assignment. | */
   MACE_POINT_DISCOVERY_NEW=1, /* The point reported is newly discovered relative to the list initially recieved. | */
   MACE_POINT_DISCOVERY_ENUM_END=2, /*  | */
} MACE_POINT_DISCOVERY;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MACE_STRESS_THRESHOLD
#define HAVE_ENUM_MACE_STRESS_THRESHOLD
typedef enum MACE_STRESS_THRESHOLD
{
   MACE_STRESS_THRESHOLD_STRESS_MAX=0, /* State of the stress point has been confirmed at a maximum. | */
   MACE_STRESS_THRESHOLD_AMBIGUOUS=1, /* State of the stress point has yet to be confirmed. | */
   MACE_STRESS_THRESHOLD_STRESS_MIN=2, /* State of the stress point has been confirmed at a minimum. | */
   MACE_STRESS_THRESHOLD_ENUM_END=3, /*  | */
} MACE_STRESS_THRESHOLD;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_mace_mission_request_partial_list.h"
#include "./mavlink_msg_mace_mission_write_partial_list.h"
#include "./mavlink_msg_mace_mission_item.h"
#include "./mavlink_msg_mace_mission_request.h"
#include "./mavlink_msg_mace_mission_set_current.h"
#include "./mavlink_msg_mace_mission_current.h"
#include "./mavlink_msg_mace_mission_request_list.h"
#include "./mavlink_msg_mace_mission_count.h"
#include "./mavlink_msg_mace_mission_clear_all.h"
#include "./mavlink_msg_mace_mission_item_reached.h"
#include "./mavlink_msg_mace_mission_ack.h"
#include "./mavlink_msg_mace_roi_ag.h"

// base include
#include "../ardupilotmega/ardupilotmega.h"

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS, MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS, MAVLINK_MESSAGE_INFO_MEMINFO, MAVLINK_MESSAGE_INFO_AP_ADC, MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE, MAVLINK_MESSAGE_INFO_DIGICAM_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE, MAVLINK_MESSAGE_INFO_MOUNT_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_STATUS, MAVLINK_MESSAGE_INFO_FENCE_POINT, MAVLINK_MESSAGE_INFO_FENCE_FETCH_POINT, MAVLINK_MESSAGE_INFO_FENCE_STATUS, MAVLINK_MESSAGE_INFO_AHRS, MAVLINK_MESSAGE_INFO_SIMSTATE, MAVLINK_MESSAGE_INFO_HWSTATUS, MAVLINK_MESSAGE_INFO_RADIO, MAVLINK_MESSAGE_INFO_LIMITS_STATUS, MAVLINK_MESSAGE_INFO_WIND, MAVLINK_MESSAGE_INFO_DATA16, MAVLINK_MESSAGE_INFO_DATA32, MAVLINK_MESSAGE_INFO_DATA64, MAVLINK_MESSAGE_INFO_DATA96, MAVLINK_MESSAGE_INFO_RANGEFINDER, MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL, MAVLINK_MESSAGE_INFO_RALLY_POINT, MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT, MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS, MAVLINK_MESSAGE_INFO_AHRS2, MAVLINK_MESSAGE_INFO_CAMERA_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK, MAVLINK_MESSAGE_INFO_BATTERY2, MAVLINK_MESSAGE_INFO_AHRS3, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST, MAVLINK_MESSAGE_INFO_REMOTE_LOG_DATA_BLOCK, MAVLINK_MESSAGE_INFO_REMOTE_LOG_BLOCK_STATUS, MAVLINK_MESSAGE_INFO_LED_CONTROL, MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS, MAVLINK_MESSAGE_INFO_MAG_CAL_REPORT, MAVLINK_MESSAGE_INFO_EKF_STATUS_REPORT, MAVLINK_MESSAGE_INFO_PID_TUNING, MAVLINK_MESSAGE_INFO_GIMBAL_REPORT, MAVLINK_MESSAGE_INFO_GIMBAL_CONTROL, MAVLINK_MESSAGE_INFO_GIMBAL_TORQUE_CMD_REPORT, MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT, MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_GET_RESPONSE, MAVLINK_MESSAGE_INFO_GOPRO_SET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_SET_RESPONSE, MAVLINK_MESSAGE_INFO_RPM, MAVLINK_MESSAGE_INFO_MACE_MISSION_REQUEST_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MACE_MISSION_WRITE_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MACE_MISSION_ITEM, MAVLINK_MESSAGE_INFO_MACE_MISSION_REQUEST, MAVLINK_MESSAGE_INFO_MACE_MISSION_SET_CURRENT, MAVLINK_MESSAGE_INFO_MACE_MISSION_CURRENT, MAVLINK_MESSAGE_INFO_MACE_MISSION_REQUEST_LIST, MAVLINK_MESSAGE_INFO_MACE_MISSION_COUNT, MAVLINK_MESSAGE_INFO_MACE_MISSION_CLEAR_ALL, MAVLINK_MESSAGE_INFO_MACE_MISSION_ITEM_REACHED, MAVLINK_MESSAGE_INFO_MACE_MISSION_ACK, MAVLINK_MESSAGE_INFO_MACE_ROI_AG, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ_REPLY, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE_REPLY, MAVLINK_MESSAGE_INFO_ADAP_TUNING, MAVLINK_MESSAGE_INFO_VISION_POSITION_DELTA}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_MACE_H
