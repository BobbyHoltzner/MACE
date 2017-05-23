#include "action_mission_command.h"
namespace CommandItem {

Data::CommandItemType ActionMissionCommand::getCommandType() const
{
    return Data::CommandItemType::CI_ACT_MISSIONCOMMAND;
}

std::string ActionMissionCommand::getDescription() const
{
    return "This influences whether or not the vehicle should start or pause the current mission state.";
}

bool ActionMissionCommand::hasSpatialInfluence() const
{
    return false;
}

ActionMissionCommand::ActionMissionCommand()
{

}

ActionMissionCommand::ActionMissionCommand(const ActionMissionCommand &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionMissionCommand::ActionMissionCommand(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

}
