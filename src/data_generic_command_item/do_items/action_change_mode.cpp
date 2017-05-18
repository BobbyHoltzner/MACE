#include "action_change_mode.h"

namespace CommandItem {

Data::CommandItemType ActionChangeMode::getCommandType() const
{
    return Data::CommandItemType::CI_ACT_CHANGEMODE;
}

std::string ActionChangeMode::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool ActionChangeMode::hasSpatialInfluence() const
{
    return false;
}

ActionChangeMode::ActionChangeMode()
{

}

ActionChangeMode::ActionChangeMode(const ActionChangeMode &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionChangeMode::ActionChangeMode(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

}

