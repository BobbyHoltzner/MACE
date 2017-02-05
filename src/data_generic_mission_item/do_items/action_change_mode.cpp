#include "action_change_mode.h"

namespace MissionItem {

MissionItemType ActionChangeMode::getMissionType()
{
    return MissionItemType::ARM;
}

std::string ActionChangeMode::getDescription()
{
    return "This changes the mode of the vehicle";
}

bool ActionChangeMode::hasSpatialInfluence()
{
    return false;
}

}

