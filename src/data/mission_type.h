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
    PROPOSED,
    TRANSMITTED
};

inline std::string MissionTypeStateToString(const MissionTypeState &cmdType) {
    switch (cmdType) {
    case MissionTypeState::CURRENT:
        return "CURRENT";
    case MissionTypeState::PROPOSED:
        return "PROPOSED";
    case MissionTypeState::TRANSMITTED:
        return "TRANSMITTED";
    default:
        throw std::runtime_error("Unknown MissionTypeState seen");
    }
}

inline MissionTypeState MissionTypeStateFromString(const std::string &str) {
    if(str == "CURRENT")
        return MissionTypeState::CURRENT;
    if(str == "PROPOSED")
        return MissionTypeState::PROPOSED;
    if(str == "TRANSMITTED")
        return MissionTypeState::TRANSMITTED;
    throw std::runtime_error("Unknown string MissionTypeState seen");
}

enum class MissionType : uint8_t
{
    ACTION,
    ALL,
    AUTO,
    GUIDED
};

inline std::string MissionTypeToString(const MissionType &cmdType) {
    switch (cmdType) {
    case MissionType::ACTION:
        return "ACTION";
    case MissionType::ALL:
        return "ALL";
    case MissionType::AUTO:
        return "AUTO";
    case MissionType::GUIDED:
        return "GUIDED";
    default:
        throw std::runtime_error("Unknown MissionType seen");
    }
}

inline MissionType MissionTypeFromString(const std::string &str) {
    if(str == "ACTION")
        return MissionType::ACTION;
    if(str == "ALL")
        return MissionType::ALL;
    if(str == "AUTO")
        return MissionType::AUTO;
    if(str == "GUIDED")
        return MissionType::GUIDED;
    throw std::runtime_error("Unknown string MissionType seen");
}

}

#endif // MISSION_TYPE_H
