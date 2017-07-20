#include "state_attitude.h"

using namespace DataState;

StateAttitude::StateAttitude():
    roll(0.0),pitch(0.0),yaw(0.0),rollRate(0.0),pitchRate(0.0),yawRate(0.0)
{

}

StateAttitude::StateAttitude(const StateAttitude &attitude)
{
    this->roll = attitude.roll;
    this->rollRate = attitude.rollRate;

    this->pitch = attitude.pitch;
    this->pitchRate = attitude.pitchRate;

    this->yaw = attitude.yaw;
    this->yawRate = attitude.yawRate;
}

void StateAttitude::setAttitude(const double &roll, const double &pitch, const double &yaw)
{
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
}

void StateAttitude::setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate)
{
    this->rollRate = rollRate;
    this->pitchRate =pitchRate;
    this->yawRate = yawRate;
}
