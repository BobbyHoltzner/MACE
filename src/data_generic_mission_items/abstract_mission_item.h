#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include "data/vehicle_command_types.h"

namespace DataGenericMission {

enum class MissionItemTypes{
    WAYPOINT
};

class AbstractMissionItem
{
public:
    virtual Data::VehicleCommandTypes getCommandType() const = 0;

    virtual MissionItemTypes getMissionType() const = 0;

    virtual std::string getDescription() const = 0;
};

} //end of namespace DataGenericMission
#endif // ABSTRACT_MISSION_ITEM_H
