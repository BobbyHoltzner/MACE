/** @file
 *  @brief MAVLink comm protocol generated from boundary.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_BOUNDARY_H
#define MACE_BOUNDARY_H

#ifndef MACE_H
    #error Wrong include order: MACE_BOUNDARY.H MUST NOT BE DIRECTLY USED. Include mace.h from the same directory instead or set ALL AND EVERY defines from MACE.H manually accordingly, including the #define MACE_H call.
#endif

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 3

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MACE_MESSAGE_LENGTHS
#define MACE_MESSAGE_LENGTHS {}
#endif

#ifndef MACE_MESSAGE_CRCS
#define MACE_MESSAGE_CRCS {{130, 154, 3, 0, 0, 0}, {131, 36, 3, 0, 0, 0}, {132, 247, 3, 0, 0, 0}, {133, 168, 5, 0, 0, 0}, {134, 205, 5, 0, 0, 0}, {135, 12, 18, 0, 0, 0}}
#endif

#include "../mace_protocol.h"

#define MACE_ENABLED_BOUNDARY

// ENUM DEFINITIONS


/** @brief Type of boundary items being requested/sent in boundary protocol. */
#ifndef HAVE_ENUM_BOUNDARY_TYPE
#define HAVE_ENUM_BOUNDARY_TYPE
typedef enum BOUNDARY_TYPE
{
   OPERATIONAL_FENCE=0, /* Items are operational boundary coniditions constraining the AO. Often this will be used to describe the general allowable flight area for all vehicles. | */
   RESOURCE_FENCE=1, /* Items are operational boundary conditions that may be a subset of the operational fence. Often used while tasking resources and determining the general area each agent should be concerned with. | */
   GENERIC_POLYGON=2, /* Items are described for a general polygon. | */
   BOUNDARY_TYPE_ENUM_END=3, /*  | */
} BOUNDARY_TYPE;
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
#include "./mace_msg_new_boundary_object.h"
#include "./mace_msg_ack_rxboundary.h"
#include "./mace_msg_boundary_request_list.h"
#include "./mace_msg_boundary_count.h"
#include "./mace_msg_boundary_request_item.h"
#include "./mace_msg_boundary_item.h"

// base include
#include "../common/common.h"

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 3

#if MACE_THIS_XML_IDX == MACE_PRIMARY_XML_IDX
# define MACE_MESSAGE_INFO {MACE_MESSAGE_INFO_NEW_BOUNDARY_OBJECT, MACE_MESSAGE_INFO_ACK_RXBOUNDARY, MACE_MESSAGE_INFO_BOUNDARY_REQUEST_LIST, MACE_MESSAGE_INFO_BOUNDARY_COUNT, MACE_MESSAGE_INFO_BOUNDARY_REQUEST_ITEM, MACE_MESSAGE_INFO_BOUNDARY_ITEM}
# if MACE_COMMAND_24BIT
#  include "../mace_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_BOUNDARY_H
