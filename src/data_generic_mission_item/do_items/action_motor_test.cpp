#include "action_motor_test.h"

namespace MissionItem {

Data::MissionItemType ActionMotorTest::getMissionType() const
{
    return Data::MissionItemType::MI_ACT_MOTORTEST;
}

std::string ActionMotorTest::getDescription() const
{
    return "This tests the operation of vehicle motors";
}

bool ActionMotorTest::hasSpatialMissionInfluence() const
{
    return false;
}

ActionMotorTest::ActionMotorTest()
{

}
ActionMotorTest::ActionMotorTest(const int &vehicleID)
{
    this->m_VehicleID = vehicleID;
}

}
