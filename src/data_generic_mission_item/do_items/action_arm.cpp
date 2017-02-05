#include "action_arm.h"

namespace MissionItem {

MissionItemType ActionArm::getMissionType()
{
    return MissionItemType::ARM;
}

std::string ActionArm::getDescription()
{
    return "This arms the vehicle";
}

bool ActionArm::hasSpatialInfluence()
{
    return false;
}

}
