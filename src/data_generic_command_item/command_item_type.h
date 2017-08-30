#ifndef COMMAND_ITEM_TYPE_H
#define COMMAND_ITEM_TYPE_H

#include <string>
#include <stdexcept>

namespace CommandItem
{

enum class COMMANDITEM : uint8_t{
    CI_NAV_HOME = 0,
    CI_NAV_LAND=1, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_TIME=2, /* Loiter around this CISSION for X seconds |Seconds (decimal)| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_TURNS=3, /* Loiter around this CISSION for X turns |Turns| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_UNLIM=4, /* Loiter around this CISSION an unliCIted amount of time |Empty| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    CI_NAV_RETURN_TO_LAUNCH=5, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CI_NAV_TAKEOFF=6, /* Takeoff from ground / hand |CInimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_NAV_WAYPOINT=7, /* Navigate to CISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at CISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the CISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at CISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_ACT_ARM = 8,
    CI_ACT_CHANGESPEED = 9,
    CI_ACT_CHANGEMODE = 10,
    CI_ACT_MOTORTEST = 11,
    CI_ACT_MISSIONCOMMAND = 12,
    CI_UNKNOWN = 14,
    COMMANDITEMEND = 15
};

inline std::string CommandItemToString(const COMMANDITEM &commandItemType) {
    switch (commandItemType) {
    case COMMANDITEM::CI_NAV_LAND:
        return "CI_NAV_LAND";
    case COMMANDITEM::CI_NAV_LOITER_TIME:
        return "CI_NAV_LOITER_TIME";
    case COMMANDITEM::CI_NAV_LOITER_TURNS:
        return "CI_NAV_LOITER_TURNS";
    case COMMANDITEM::CI_NAV_LOITER_UNLIM:
        return "CI_NAV_LOITER_UNLIM";
    case COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH:
        return "CI_NAV_RETURN_TO_LAUNCH";
    case COMMANDITEM::CI_NAV_TAKEOFF:
        return "CI_NAV_TAKEOFF";
    case COMMANDITEM::CI_NAV_WAYPOINT:
        return "CI_NAV_WAYPOINT";
    case COMMANDITEM::CI_ACT_ARM:
        return "CI_ACT_ARM";
    case COMMANDITEM::CI_ACT_CHANGESPEED:
        return "CI_ACT_CHANGESPEED";
    case COMMANDITEM::CI_ACT_CHANGEMODE:
        return "CI_ACT_CHANGEMODE";
    case COMMANDITEM::CI_ACT_MOTORTEST:
        return "CI_ACT_MOTORTEST";
    case COMMANDITEM::CI_ACT_MISSIONCOMMAND:
        return "CI_ACT_MISSIONCOMMAND";
    case COMMANDITEM::CI_UNKNOWN:
        return "CI_UNKNOWN";
    default:
        throw std::runtime_error("Unknown mission item enum seen");
    }
}

inline COMMANDITEM CommandItemFromString(const std::string &str) {
    if(str == "CI_NAV_LAND")
        return COMMANDITEM::CI_NAV_LAND;
    if(str == "CI_NAV_LOITER_TIME")
        return COMMANDITEM::CI_NAV_LOITER_TIME;
    if(str == "CI_NAV_LOITER_TURNS")
        return COMMANDITEM::CI_NAV_LOITER_TURNS;
    if(str == "CI_NAV_LOITER_UNLIM")
        return COMMANDITEM::CI_NAV_LOITER_UNLIM;
    if(str == "CI_NAV_RETURN_TO_LAUNCH")
        return COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
    if(str == "CI_NAV_TAKEOFF")
        return COMMANDITEM::CI_NAV_TAKEOFF;
    if(str == "CI_NAV_WAYPOINT")
        return COMMANDITEM::CI_NAV_WAYPOINT;
    if(str == "CI_ACT_ARM")
        return COMMANDITEM::CI_ACT_ARM;
    if(str == "CI_ACT_CHANGESPEED")
        return COMMANDITEM::CI_ACT_CHANGESPEED;
    if(str == "CI_ACT_CHANGEMODE")
        return COMMANDITEM::CI_ACT_CHANGEMODE;
    if(str == "CI_ACT_MOTORTEST")
        return COMMANDITEM::CI_ACT_MOTORTEST;
    if(str == "CI_ACT_MISSIONCOMMAND")
        return COMMANDITEM::CI_ACT_MISSIONCOMMAND;
    if(str == "CI_UNKNOWN")
        return COMMANDITEM::CI_UNKNOWN;
    throw std::runtime_error("Unknown mission item string seen");
}

} //end of namespace Data

#endif // COMMAND_ITEM_TYPE_H
