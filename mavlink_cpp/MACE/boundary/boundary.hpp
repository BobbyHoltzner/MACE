/** @file
 *	@brief MAVLink comm protocol generated from boundary.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace boundary {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 6> MESSAGE_ENTRIES {{ {130, 154, 3, 0, 0, 0}, {131, 47, 4, 0, 0, 0}, {132, 247, 3, 0, 0, 0}, {133, 168, 5, 0, 0, 0}, {134, 205, 5, 0, 0, 0}, {135, 12, 18, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief Type of boundary items being requested/sent in boundary protocol. */
enum class BOUNDARY_TYPE : uint8_t
{
    OPERATIONAL_FENCE=0, /* Items are operational boundary coniditions constraining the AO. Often this will be used to describe the general allowable flight area for all vehicles. | */
    RESOURCE_FENCE=1, /* Items are operational boundary conditions that may be a subset of the operational fence. Often used while tasking resources and determining the general area each agent should be concerned with. | */
    GENERIC_POLYGON=2, /* Items are described for a general polygon. | */
};

//! BOUNDARY_TYPE ENUM_END
constexpr auto BOUNDARY_TYPE_ENUM_END = 3;

/** @brief result in a mavlink mission ack */
enum class BOUNDARY_RESULT : uint8_t
{
    ACCEPTED=0, /* boundary accepted OK | */
    ERROR=1, /* generic error / not accepting mission commands at all right now | */
    UNSUPPORTED_FRAME=2, /* coordinate frame is not supported | */
    UNSUPPORTED=3, /* boundary is not supported | */
    NO_SPACE=4, /* boundary item exceeds storage space | */
    INVALID=5, /* one of the parameters has an invalid value | */
    INVALID_SEQUENCE=6, /* received boundary item out of sequence | */
    DENIED=7, /* not accepting any boundary commands from this communication partner | */
    DOES_NOT_EXIST=15, /* the requested boundary with the associated key does not exist. | */
};

//! BOUNDARY_RESULT ENUM_END
constexpr auto BOUNDARY_RESULT_ENUM_END = 16;


} // namespace boundary
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_new_boundary_object.hpp"
#include "./mavlink_msg_boundary_ack.hpp"
#include "./mavlink_msg_boundary_request_list.hpp"
#include "./mavlink_msg_boundary_count.hpp"
#include "./mavlink_msg_boundary_request_item.hpp"
#include "./mavlink_msg_boundary_item.hpp"

// base include
#include "../common/common.hpp"
