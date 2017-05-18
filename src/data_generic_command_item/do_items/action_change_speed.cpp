#include "action_change_speed.h"

namespace CommandItem {

Data::CommandItemType ActionChangeSpeed::getCommandType() const
{
    return Data::CommandItemType::CI_ACT_CHANGESPEED;
}

std::string ActionChangeSpeed::getDescription() const
{
    return "This changes the speed of the vehicle";
}

bool ActionChangeSpeed::hasSpatialInfluence() const
{
    return false;
}

ActionChangeSpeed::ActionChangeSpeed()
{

}

ActionChangeSpeed::ActionChangeSpeed(const ActionChangeSpeed &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionChangeSpeed::ActionChangeSpeed(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

}
