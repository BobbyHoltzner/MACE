#include "action_change_mode.h"

namespace MissionItem {

MissionItemType ActionChangeMode::getMissionType() const
{
    return MissionItemType::CHANGE_MODE;
}

std::string ActionChangeMode::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool ActionChangeMode::hasSpatialMissionInfluence() const
{
    return false;
}

ActionChangeMode::ActionChangeMode()
{

}

ActionChangeMode::ActionChangeMode(const int &vehicleID, const std::string &mode)
{
    m_VehicleID = vehicleID;
    m_CommandVehicleMode = mode;
}

ActionChangeMode::ActionChangeMode(const ActionChangeMode &actionChangeMode)
{
    this->m_VehicleID = actionChangeMode.m_VehicleID;
    this->m_CommandVehicleMode = actionChangeMode.m_CommandVehicleMode;
}

}

