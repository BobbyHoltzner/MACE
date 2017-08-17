#include "action_arm.h"

namespace CommandItem {

COMMANDITEM ActionArm::getCommandType() const
{
    return COMMANDITEM::CI_ACT_ARM;
}

std::string ActionArm::getDescription() const
{
    return "This arms the vehicle";
}

bool ActionArm::hasSpatialInfluence() const
{
    return false;
}

ActionArm::ActionArm()
{

}

ActionArm::ActionArm(const ActionArm &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionArm::ActionArm(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}




}
