#include "action_change_speed.h"

namespace MissionItem {

MissionItemType ActionChangeSpeed::getMissionType() const
{
    return MissionItemType::CHANGE_SPEED;
}

std::string ActionChangeSpeed::getDescription() const
{
    return "This changes the speed of the vehicle";
}

bool ActionChangeSpeed::hasSpatialMissionInfluence() const
{
    return false;
}

ActionChangeSpeed::ActionChangeSpeed()
{

}
ActionChangeSpeed::ActionChangeSpeed(const int &vehicleID)
{
    this->m_VehicleID = vehicleID;
}

}
