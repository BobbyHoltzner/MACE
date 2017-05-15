#include "action_change_speed.h"

namespace MissionItem {

Data::MissionItemType ActionChangeSpeed::getMissionType() const
{
    return Data::MissionItemType::MI_ACT_CHANGESPEED;
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
