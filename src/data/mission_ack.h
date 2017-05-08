#ifndef MISSION_ACK_H
#define MISSION_ACK_H

namespace Data
{

enum class MissionACK
{
    ACKNOWLEDGED,
    REJECTED,
    ACKNOWLEDGED_MODIFIED_TYPE,
    ACKNOWLEDGED_MODIFIED_ID,
    UNKNOWN
};

}

#endif // MISSION_ACK_H
