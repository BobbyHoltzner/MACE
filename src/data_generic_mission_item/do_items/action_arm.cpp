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

ActionArm::ActionArm()
{

}

ActionArm::ActionArm(const int &vehicleID, const bool &arm)
{
    m_ActionArm = arm;
    m_VehicleID = vehicleID;
}


ActionArm::ActionArm(const ActionArm &actionArm)
{
    this->m_ActionArm = actionArm.m_ActionArm;
    this->m_VehicleID = actionArm.m_VehicleID;
}

}
