#ifndef GENERAL_MISSION_ITEM_H
#define GENERAL_MISSION_ITEM_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class MissionItemTypes{
    LAND,
    TAKEOFF,
    WAYPOINT,
    GENERAL
};

class GeneralMissionItem
{
public:
    virtual CommandTypes getCommandType() const;

    virtual MissionItemTypes getMissionType() const;

    virtual std::string getDescription() const;
};

} //end of namespace DataVehicleCommands

#endif // GENERAL_MISSION_ITEM_H
