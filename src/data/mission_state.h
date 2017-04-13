#ifndef MISSION_STATE_H
#define MISSION_STATE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class MissionState{
    ROUTING,
    HUNTING,
    ACHIEVED
};

inline std::string MissionStateToString(const MissionState &state) {
    switch (state) {
    case MissionState::ROUTING:
        return "ROUTING";
    case MissionState::HUNTING:
        return "HUNTING";
    case MissionState::ACHIEVED:
        return "ACHIEVED";
    default:
        throw std::runtime_error("Unknown mission state seen");
    }
}

inline MissionState MissionStateFromString(const std::string &str) {
    if(str == "ROUTING")
        return MissionState::ROUTING;
    if(str == "HUNTING")
        return MissionState::HUNTING;
    if(str == "ACHIEVED")
        return MissionState::ACHIEVED;

    throw std::runtime_error("Unknown mission state seen");
}


} //end of namespace Data

#endif // MISSION_STATE_H
