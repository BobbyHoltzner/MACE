#include "state_mavlink_to_mace.h"

namespace DataMAVLINK{

State_MAVLINKTOMACE::State_MAVLINKTOMACE(const int &systemID) :
    mSystemID(systemID)
{

}

DataState::StateAttitude State_MAVLINKTOMACE::Attitude_MAVLINKTOMACE(const mavlink_attitude_t &stateItem)
{
    DataState::StateAttitude stateAttitude;
    stateAttitude.setAttitude(stateItem.roll,stateItem.pitch,stateItem.yaw);
    stateAttitude.setAttitudeRates(stateItem.rollspeed,stateItem.pitchspeed,stateItem.yawspeed);
    return stateAttitude;
}

DataState::StateGlobalPosition State_MAVLINKTOMACE::GlobalPosition_MAVLINKTOMACE(const mavlink_global_position_int_t &stateItem)
{
    DataState::StateGlobalPosition stateGlobalPosition;
    stateGlobalPosition.setX(stateItem.lat/pow(10,7));
    stateGlobalPosition.setY(stateItem.lon/pow(10,7));
    stateGlobalPosition.setZ(stateItem.alt/1000);
    return stateGlobalPosition;
}

DataState::StateLocalPosition State_MAVLINKTOMACE::LocalPosition_MAVLINKTOMACE(const mavlink_local_position_ned_t &stateItem)
{
    DataState::StateLocalPosition stateLocalPosition;
    stateLocalPosition.setPositionX(stateItem.x);
    stateLocalPosition.setPositionY(stateItem.y);
    stateLocalPosition.setPositionZ(stateItem.z);
    return stateLocalPosition;
}


} //end of namespace DataMAVLINK
