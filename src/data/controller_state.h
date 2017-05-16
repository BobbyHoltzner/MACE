#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class ControllerState{
    TRACKING,
    HUNTING,
    ACHIEVED
};

inline std::string GuidedStateToString(const ControllerState &state) {
    switch (state) {
    case ControllerState::TRACKING:
        return "TRACKING";
    case ControllerState::HUNTING:
        return "HUNTING";
    case ControllerState::ACHIEVED:
        return "ACHIEVED";
    default:
        throw std::runtime_error("Unknown guided state seen");
    }
}

inline ControllerState GuidedStateFromString(const std::string &str) {
    if(str == "TRACKING")
        return ControllerState::TRACKING;
    if(str == "HUNTING")
        return ControllerState::HUNTING;
    if(str == "ACHIEVED")
        return ControllerState::ACHIEVED;

    throw std::runtime_error("Unknown guided state seen");
}

}

#endif // CONTROLLER_STATE_H
