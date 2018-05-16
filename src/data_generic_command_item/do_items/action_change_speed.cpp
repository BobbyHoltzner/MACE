#include "action_change_speed.h"

namespace CommandItem {

COMMANDITEM ActionChangeSpeed::getCommandType() const
{
    return COMMANDITEM::CI_ACT_CHANGESPEED;
}

std::string ActionChangeSpeed::getDescription() const
{
    return "This changes the speed of the vehicle";
}

bool ActionChangeSpeed::hasSpatialInfluence() const
{
    return false;
}

AbstractCommandItem* ActionChangeSpeed::getClone() const
{
    return (new ActionChangeSpeed(*this));
}

void ActionChangeSpeed::getClone(AbstractCommandItem** command) const
{
    *command = new ActionChangeSpeed(*this);
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
