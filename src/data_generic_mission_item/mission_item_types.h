#ifndef MISSION_ITEM_TYPES_H
#define MISSION_ITEM_TYPES_H

#include <string>
#include <stdexcept>


namespace MissionItem {

enum class MissionItemType{
    ARM,
    CHANGE_MODE,
    CHANGE_SPEED,
    MOTOR_TEST,
    LAND,
    LOITER,
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
    case MissionItemType::CHANGE_SPEED:
        return "CHANGE_SPEED";
    case MissionItemType::MOTOR_TEST:
        return "MOTOR_TEST";
    case MissionItemType::LAND:
        return "LAND";
    case MissionItemType::LOITER:
        return "LOITER";
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
    if(str == "CHANGE_SPEED")
        return MissionItemType::CHANGE_SPEED;
    if(str == "MOTOR_TEST")
        return MissionItemType::MOTOR_TEST;
    if(str == "LAND")
        return MissionItemType::LAND;
    if(str == "LOITER")
        return MissionItemType::LOITER;
    if(str == "RTL")
        return MissionItemType::RTL;
    if(str == "TAKEOFF")
        return MissionItemType::TAKEOFF;
    if(str == "WAYPOINT")
        return MissionItemType::WAYPOINT;
    throw std::runtime_error("Unknown mission item type seen");
}

} //end of namespace MissionItem

#endif // MISSION_ITEM_TYPES_H
