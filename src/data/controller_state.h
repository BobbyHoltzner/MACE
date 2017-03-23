#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class GuidedState{
    TRACKING,
    HUNTING,
    ACHIEVED
};

inline std::string GuidedStateToString(const GuidedState &state) {
    switch (state) {
    case GuidedState::TRACKING:
        return "TRACKING";
    case GuidedState::HUNTING:
        return "HUNTING";
    case CoordinateFrame::ACHIEVED:
        return "ACHIEVED";
    default:
        throw std::runtime_error("Unknown guided state seen");
    }
}

inline GuidedState GuidedStateFromString(const std::string &str) {
    if(str == "TRACKING")
        return GuidedState::TRACKING;
    if(str == "HUNTING")
        return GuidedState::HUNTING;
    if(str == "ACHIEVED")
        return GuidedState::ACHIEVED;

    throw std::runtime_error("Unknown guided state seen");
}


enum class PlanningHorizon{
    IMMEDIATE,
    LOCAL,
    GLOBAL
};

inline std::string PlanningHorizonToString(const PlanningHorizon &state) {
    switch (state) {
    case PlanningHorizon::IMMEDIATE:
        return "IMMEDIATE";
    case PlanningHorizon::LOCAL:
        return "LOCAL";
    case PlanningHorizon::GLOBAL:
        return "GLOBAL";
    default:
        throw std::runtime_error("Unknown planning horizon seen");
    }
}

inline PlanningHorizon PlanningHorizonFromString(const std::string &str) {
    if(str == "IMMEDIATE")
        return PlanningHorizon::IMMEDIATE;
    if(str == "LOCAL")
        return PlanningHorizon::LOCAL;
    if(str == "GLOBAL")
        return PlanningHorizon::GLOBAL;

    throw std::runtime_error("Unknown planning horizon seen");
}

} //end of namespace Data

#endif // CONTROLLER_STATE_H