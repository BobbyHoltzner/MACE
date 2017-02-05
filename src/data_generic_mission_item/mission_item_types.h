#ifndef MISSION_ITEM_TYPES_H
#define MISSION_ITEM_TYPES_H

#include <string>
#include <stdexcept>


namespace MissionItem {

enum class MissionItemType{
    ARM,
    CHANGE_MODE,
    LAND,
    RTL,
    TAKEOFF,
    WAYPOINT
};


inline std::string MissionTypeToString(const MissionItemType &cmdType) {
    switch (cmdType) {
    case MissionItemType::ARM:
        return "ARM";
    case MissionItemType::CHANGE_MODE:
        return "CHANGE_MODE";
    case MissionItemType::LAND:
        return "LAND";
    case MissionItemType::RTL:
        return "RTL";
    case MissionItemType::TAKEOFF:
        return "TAKEOFF";
    case MissionItemType::WAYPOINT:
        return "WAYPOINT";
    default:
        throw std::runtime_error("Unknown mission item type seen");
    }
}

inline MissionItemType MissionTypeFromString(const std::string &str) {
    if(str == "ARM")
        return MissionItemType::ARM;
    if(str == "CHANGE_MODE")
        return MissionItemType::CHANGE_MODE;
    if(str == "LAND")
        return MissionItemType::LAND;
    if(str == "RTL")
        return MissionItemType::RTL;
    if(str == "TAKEOFF")
        return MissionItemType::TAKEOFF;
    if(str == "WAYPOINT")
        return MissionItemType::WAYPOINT;
    throw std::runtime_error("Unknown mission item type seen");
}

} //end of namespace DataVehicleCommands

#endif // MISSION_ITEM_TYPES_H
