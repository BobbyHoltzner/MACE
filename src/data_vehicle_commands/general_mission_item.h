#ifndef GENERAL_MISSION_ITEM_H
#define GENERAL_MISSION_ITEM_H

#include "command_types.h"
#include "mission_item_types.h"

namespace DataVehicleCommands {

class GeneralMissionItem
{
public:
    GeneralMissionItem();

    virtual ~GeneralMissionItem();

    virtual CommandTypes getCommandType() const = 0;

    virtual MissionItemTypes getMissionType() const = 0;

    virtual std::string getDescription() const = 0;
};

} //end of namespace DataVehicleCommands

#endif // GENERAL_MISSION_ITEM_H
