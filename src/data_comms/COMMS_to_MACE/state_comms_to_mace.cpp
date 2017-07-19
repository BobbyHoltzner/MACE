#include "state_comms_to_mace.h"

namespace DataCOMMS{

State_COMMSTOMACE::State_COMMSTOMACE()
{

}

DataState::StateAttitude State_COMMSTOMACE::Attitude_COMMSTOMACE(const mace_attitude_t &stateItem, const int &systemID)
{
    UNUSED(systemID);
    DataState::StateAttitude stateAttitude;
    stateAttitude.setAttitude(stateItem.roll,stateItem.pitch,stateItem.yaw);
    return stateAttitude;
}

DataState::StateAttitude State_COMMSTOMACE::AttitudeRates_COMMSTOMACE(const mace_attitude_rates_t &stateItem, const int &systemID)
{
    UNUSED(systemID);
    DataState::StateAttitude stateAttitude;
    stateAttitude.setAttitudeRates(stateItem.rollspeed,stateItem.pitchspeed,stateItem.yawspeed);
    return stateAttitude;
}

DataState::StateGlobalPosition State_COMMSTOMACE::GlobalPosition_COMMSTOMACE(const mace_global_position_int_t &stateItem, const int &systemID)
{
    UNUSED(systemID);
    DataState::StateGlobalPosition stateGlobalPosition;
    stateGlobalPosition.setLatitude(stateItem.lat/pow(10,7));
    stateGlobalPosition.setLongitude(stateItem.lon/pow(10,7));
    stateGlobalPosition.setAltitude(stateItem.alt/1000.0);
    return stateGlobalPosition;
}

DataState::StateLocalPosition State_COMMSTOMACE::LocalPosition_COMMSTOMACE(const mace_local_position_ned_t &stateItem, const int &systemID)
{
    UNUSED(systemID);
    DataState::StateLocalPosition stateLocalPosition;
    stateLocalPosition.setX(stateItem.x);
    stateLocalPosition.setY(stateItem.y);
    stateLocalPosition.setZ(stateItem.z);
    return stateLocalPosition;
}

} //end of namespace DataCOMMS
