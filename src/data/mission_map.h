#ifndef MISSION_MAP_H
#define MISSION_MAP_H

#include <stdint.h>

namespace Data
{

enum class MissionMap
{
    PROPOSED,
    CURRENT
};

enum class MissionType : uint8_t
{
    AUTO_PROPOSED,
    AUTO_CURRENT,
    GUIDED_PROPOSED,
    GUIDED_CURRENT
};

}

#endif // MISSION_MAP_H
