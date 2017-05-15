#include "action_change_mode.h"

namespace MissionItem {

Data::MissionItemType ActionChangeMode::getMissionType() const
{
    return Data::MissionItemType::MI_ACT_CHANGEMODE;
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
    this->operator =(actionChangeMode);
}

}

