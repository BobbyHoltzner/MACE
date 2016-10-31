#include "arducopter_attitude.h"
#include <iostream>

namespace Data {

ArducopterAttitude::ArducopterAttitude()
    :ArducopterData()
{
    mTimeboot = 0.0;
    mRoll = 0.0;
    mPitch = 0.0;
    mYaw = 0.0;
    mRollSpeed = 0.0;
    mPitchSpeed = 0.0;
    mYawSpeed = 0.0;
}

ArducopterMessageDef ArducopterAttitude::getMessageDef() const
{
    return MESSAGE_ATTITUDE;
}

std::string ArducopterAttitude::getMessageDescription() const
{
    return "This is a message containing arducopter ATTITUDE data";
}

ArducopterAttitude& ArducopterAttitude::operator =(const ArducopterAttitude &inArducopterAttitude)
{
    //Do this for all of the member objects
}

bool ArducopterAttitude::operator ==(const ArducopterAttitude& rhs) const
{
    return false;
}

bool ArducopterAttitude::operator !=(const ArducopterAttitude& rhs) const
{
    return false;
}

void ArducopterAttitude::updateAttitudeInformation(const mavlink_attitude_t &msg)
{
    std::cout<<"I saw an update attitude message with a roll value of: "<<msg.roll<<std::endl;
    mTimeboot = msg.time_boot_ms;
    mRoll = msg.roll;
    mPitch = msg.pitch;
    mYaw = msg.yaw;
    mRollSpeed = msg.rollspeed;
    mPitchSpeed = msg.pitchspeed;
    mYawSpeed = msg.yawspeed;
    std::cout<<"The updated member now holds: "<<mRoll<<std::endl;

}

}




