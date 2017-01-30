#ifndef GENERAL_MISSION_ITEM_H
#define GENERAL_MISSION_ITEM_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class MissionCommandTypes{
    WAYPOINT
};

class AbstractMissionCommand
{
public:
    virtual CommandTypes getCommandType() const = 0;

    virtual MissionCommandTypes getMissionType() const = 0;

    virtual std::string getDescription() const = 0;
};

} //end of namespace DataVehicleCommands

#endif // GENERAL_MISSION_ITEM_H
