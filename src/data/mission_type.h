#ifndef MISSION_TYPE_H
#define MISSION_TYPE_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace Data
{

enum class MissionTXState : uint8_t
{
    CURRENT,
    ONBOARD,
    OUTDATED,
    PROPOSED,
    RECEIVED
};

inline std::string MissionTXStateToString(const MissionTXState &cmdType) {
    switch (cmdType) {
    case MissionTXState::CURRENT:
        return "CURRENT";
    case MissionTXState::ONBOARD:
        return "ONBOARD";
    case MissionTXState::OUTDATED:
        return "OUTDATED";
    case MissionTXState::PROPOSED:
        return "PROPOSED";
    case MissionTXState::RECEIVED:
        return "RECEIVED";
    default:
        throw std::runtime_error("Unknown MissionTXState seen");
    }
}

inline MissionTXState MissionTXStateFromString(const std::string &str) {
    if(str == "CURRENT")
        return MissionTXState::CURRENT;
    if(str == "ONBOARD")
        return MissionTXState::ONBOARD;
    if(str == "OUTDATED")
        return MissionTXState::OUTDATED;
    if(str == "PROPOSED")
        return MissionTXState::PROPOSED;
    if(str == "RECEIVED")
        return MissionTXState::RECEIVED;
    throw std::runtime_error("Unknown string MissionTXState seen");
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

inline MissionType MissionTypeFromString(const std::string &str) {
    if(str == "AUTO")
        return MissionType::AUTO;
    if(str == "GUIDED")
        return MissionType::GUIDED;
    if(str == "ROI")
        return MissionType::ROI;
    if(str == "FENCE")
        return MissionType::FENCE;
    if(str == "RALLY")
        return MissionType::RALLY;
    if(str == "ALL")
        return MissionType::ALL;
    throw std::runtime_error("Unknown string MissionType seen");
}

inline std::string MissionTypeToString(const MissionType &type) {
    switch (type) {
    case MissionType::AUTO:
        return "AUTO";
    case MissionType::GUIDED:
        return "GUIDED";
    case MissionType::ROI:
        return "ROI";
    case MissionType::FENCE:
        return "FENCE";
    case MissionType::RALLY:
        return "RALLY";
    case MissionType::ALL:
        return "ALL";
    default:
        throw std::runtime_error("Unknown MissionType seen");
    }
}

}

#endif // MISSION_TYPE_H
