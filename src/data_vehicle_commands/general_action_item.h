#ifndef GENERAL_ACTION_ITEM_H
#define GENERAL_ACTION_ITEM_H

#include "command_types.h"
#include "mission_item_types.h"

namespace DataVehicleCommands {

class GeneralActionItem
{
public:
    GeneralActionItem();

    virtual ~GeneralActionItem();

    virtual CommandTypes getCommandType() const = 0;

    virtual MissionItemTypes getMissionType() const = 0;

    virtual std::string getDescription() const = 0;
};

} //end of namespace DataVehicleCommands

#endif // GENERAL_ACTION_ITEM_H
