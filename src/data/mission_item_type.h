#ifndef MISSION_ITEM_TYPE_H
#define MISSION_ITEM_TYPE_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class MissionItemType{
    MI_NAV_LAND=0, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    MI_NAV_LOITER_TIME=1, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    MI_NAV_LOITER_TURNS=2, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    MI_NAV_LOITER_UNLIM=3, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    MI_NAV_RETURN_TO_LAUNCH=4, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    MI_NAV_TAKEOFF=5, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    MI_NAV_WAYPOINT=6, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    MI_ACT_ARM = 7,
    MI_ACT_CHANGESPEED = 8,
    MI_ACT_CHANGEMODE = 9,
    MI_ACT_MOTORTEST = 10,
};

inline std::string MissionItemToString(const MissionItemType &missionItemType) {
    switch (missionItemType) {
    case MissionItemType::MI_NAV_LAND:
        return "MI_NAV_LAND";
    case MissionItemType::MI_NAV_LOITER_TIME:
        return "MI_NAV_LOITER_TIME";
    case MissionItemType::MI_NAV_LOITER_TURNS:
        return "MI_NAV_LOITER_TURNS";
    case MissionItemType::MI_NAV_LOITER_UNLIM:
        return "MI_NAV_LOITER_UNLIM";
    case MissionItemType::MI_NAV_RETURN_TO_LAUNCH:
        return "MI_NAV_RETURN_TO_LAUNCH";
    case MissionItemType::MI_NAV_TAKEOFF:
        return "MI_NAV_TAKEOFF";
    case MissionItemType::MI_NAV_WAYPOINT:
        return "MI_NAV_WAYPOINT";
    case MissionItemType::MI_ACT_ARM:
        return "MI_ACT_ARM";
    case MissionItemType::MI_ACT_CHANGESPEED:
        return "MI_ACT_CHANGESPEED";
    case MissionItemType::MI_ACT_CHANGEMODE:
        return "MI_ACT_CHANGEMODE";
    default:
        throw std::runtime_error("Unknown mission item enum seen");
    }
}

inline MissionItemType MissionItemFromString(const std::string &str) {
    if(str == "MI_NAV_LAND")
        return MissionItemType::MI_NAV_LAND;
    if(str == "MI_NAV_LOITER_TIME")
        return MissionItemType::MI_NAV_LOITER_TIME;
    if(str == "MI_NAV_LOITER_TURNS")
        return MissionItemType::MI_NAV_LOITER_TURNS;
    if(str == "MI_NAV_LOITER_UNLIM")
        return MissionItemType::MI_NAV_LOITER_UNLIM;
    if(str == "MI_NAV_RETURN_TO_LAUNCH")
        return MissionItemType::MI_NAV_RETURN_TO_LAUNCH;
    if(str == "MI_NAV_TAKEOFF")
        return MissionItemType::MI_NAV_TAKEOFF;
    if(str == "MI_NAV_WAYPOINT")
        return MissionItemType::MI_NAV_WAYPOINT;
    if(str == "MI_ACT_ARM")
        return MissionItemType::MI_ACT_ARM;
    if(str == "MI_ACT_CHANGESPEED")
        return MissionItemType::MI_ACT_CHANGESPEED;
    if(str == "MI_ACT_CHANGEMODE")
        return MissionItemType::MI_ACT_CHANGEMODE;
    throw std::runtime_error("Unknown mission item string seen");
}

} //end of namespace Data

#endif // MISSION_ITEM_TYPE_H
