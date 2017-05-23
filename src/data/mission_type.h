#ifndef MISSION_TYPE_H
#define MISSION_TYPE_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace Data
{

enum class MissionTypeState : uint8_t
{
    CURRENT,
    ONBOARD,
    OUTDATED,
    PROPOSED,
    RECEIVED
};

inline std::string MissionTypeStateToString(const MissionTypeState &cmdType) {
    switch (cmdType) {
    case MissionTypeState::CURRENT:
        return "CURRENT";
    case MissionTypeState::ONBOARD:
        return "ONBOARD";
    case MissionTypeState::OUTDATED:
        return "OUTDATED";
    case MissionTypeState::PROPOSED:
        return "PROPOSED";
    case MissionTypeState::RECEIVED:
        return "RECEIVED";
    default:
        throw std::runtime_error("Unknown MissionTypeState seen");
    }
}

inline MissionTypeState MissionTypeStateFromString(const std::string &str) {
    if(str == "CURRENT")
        return MissionTypeState::CURRENT;
    if(str == "ONBOARD")
        return MissionTypeState::ONBOARD;
    if(str == "OUTDATED")
        return MissionTypeState::OUTDATED;
    if(str == "PROPOSED")
        return MissionTypeState::PROPOSED;
    if(str == "RECEIVED")
        return MissionTypeState::RECEIVED;
    throw std::runtime_error("Unknown string MissionTypeState seen");
}

enum class MissionType : uint8_t
{
    AUTO,
    GUIDED,
    ROI,
    FENCE,
    RALLY,
    ALL
};

}

#endif // MISSION_TYPE_H
