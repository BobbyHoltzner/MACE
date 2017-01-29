#include "general_mission_item.h"

namespace DataVehicleCommands {

CommandTypes GeneralMissionItem::getCommandType() const
{
    return CommandTypes::MISSION;
}

MissionItemTypes GeneralMissionItem::getMissionType() const
{
    return MissionItemTypes::GENERAL;
}

std::string GeneralMissionItem::getDescription() const
{
    return "General mission item type";
}

} //end of namespace DataVehicleCommands
