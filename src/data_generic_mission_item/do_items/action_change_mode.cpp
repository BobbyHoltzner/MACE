#include "action_change_mode.h"

namespace MissionItem {

MissionItemType ActionChangeMode::getMissionType() const
{
    return MissionItemType::ARM;
}

std::string ActionChangeMode::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool ActionChangeMode::hasSpatialMissionInfluence() const
{
    return false;
}

}

