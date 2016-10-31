#ifndef ARDUCOPTERPROPERTIES_H
#define ARDUCOPTERPROPERTIES_H

#include <stdint.h>

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

#endif // ARDUCOPTERPROPERTIES_H
