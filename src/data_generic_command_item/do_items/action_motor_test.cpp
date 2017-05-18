#include "action_motor_test.h"

namespace CommandItem {

Data::CommandItemType ActionMotorTest::getCommandType() const
{
    return Data::CommandItemType::CI_ACT_MOTORTEST;
}

std::string ActionMotorTest::getDescription() const
{
    return "This tests the operation of vehicle motors";
}

bool ActionMotorTest::hasSpatialInfluence() const
{
    return false;
}

ActionMotorTest::ActionMotorTest()
{

}

ActionMotorTest::ActionMotorTest(const ActionMotorTest &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionMotorTest::ActionMotorTest(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

}
