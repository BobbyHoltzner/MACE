#include "action_arm.h"

namespace MissionItem {

MissionItemType ActionArm::getMissionType() const
{
    return MissionItemType::ARM;
}

std::string ActionArm::getDescription() const
{
    return "This arms the vehicle";
}

bool ActionArm::hasSpatialMissionInfluence() const
{
    return false;
}

}
