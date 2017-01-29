#ifndef MISSION_ITEM_TYPES_H
#define MISSION_ITEM_TYPES_H

namespace DataVehicleCommands
{


enum class MissionItemTypes{
    LAND,
    TAKEOFF,
    WAYPOINT
};

//inline std::string CommandTypeToString(const CommandTypes &cmdType) {
//    switch (frame) {
//    case CommandTypes::ACTION:
//        return "ACTION";
//    case CommandTypes::MISSION:
//        return "MISSION";
//    default:
//        throw std::runtime_error("Unknown command type seen");
//    }
//}

//inline CommandTypes CommandTypeFromString(const std::string &str) {
//    if(str == "ACTION")
//        return CommandTypes::ACTION;
//    if(str == "MISSION")
//        return CommandTypes::MISSION;

//    throw std::runtime_error("Unknown command type seen");
//}

}

#endif // MISSION_ITEM_TYPES_H
