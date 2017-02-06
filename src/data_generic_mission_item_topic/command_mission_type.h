#ifndef COMMAND_MISSION_TYPE_H
#define COMMAND_MISSION_TYPE_H

#include <string>
#include <stdexcept>

namespace MissionTopic
{

enum class MissionType{
    ACTION,
    GUIDED,
    MISSION
};

inline std::string CommandTypeToString(const MissionType &cmdType) {
    switch (cmdType) {
    case MissionType::ACTION:
        return "ACTION";
    case MissionType::GUIDED:
        return "GUIDED";
    case MissionType::MISSION:
        return "MISSION";
    default:
        throw std::runtime_error("Unknown command type seen");
    }
}

inline MissionType CommandTypeFromString(const std::string &str) {
    if(str == "ACTION")
        return MissionType::ACTION;
    if(str == "GUIDED")
        return MissionType::GUIDED;
    if(str == "MISSION")
        return MissionType::MISSION;

    throw std::runtime_error("Unknown command type seen");
}

} //end of namespace MissionTopic

#endif // COMMAND_MISSION_TYPE_H
