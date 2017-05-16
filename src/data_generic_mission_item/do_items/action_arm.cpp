#include "action_arm.h"

namespace MissionItem {

Data::MissionItemType ActionArm::getMissionType() const
{
    return Data::MissionItemType::MI_ACT_ARM;
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
    this->operator =(actionArm);
}

}
