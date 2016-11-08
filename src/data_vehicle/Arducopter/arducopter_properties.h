#ifndef ARDUCOPTERPROPERTIES_H
#define ARDUCOPTERPROPERTIES_H

#include <stdint.h>

namespace Arductoper{

enum FrameTypes{
    FRAME_UNDEFINED = 0,
    FRAME_QUADROTOR = 1,
    FRAME_TRICOPTER = 2,
    FRAME_HEXACOPTER = 3,
    FRAME_Y6 = 4,
    FRAME_OCTOCOPTER = 5,
    FRAME_HELICOPTER = 6,
    FRAME_OCTAQUAD = 7,
    FRAME_SINGLE = 8,
    FRAME_COAX = 9
};

class ArducopterProperties
{
public:
    ArducopterProperties();

private:
    uint8_t mType; /**< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM). */
    uint8_t mAutopilot; /**< Autopilot type / class. defined in MAV_AUTOPILOT ENUM. */
    uint8_t mBaseMode; /**< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h. */
    uint32_t mCustomMode; /**< A bitfield for use for autopilot-specific flags. */
    uint8_t mSystemStatus; /**< System status flag, see MAV_STATE ENUM. */
};

} //end of namespace arducopter

#endif // ARDUCOPTERPROPERTIES_H
