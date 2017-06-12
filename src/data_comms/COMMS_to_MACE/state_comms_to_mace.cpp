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
    stateGlobalPosition.latitude = stateItem.lat/pow(10,7);
    stateGlobalPosition.longitude = stateItem.lon/pow(10,7);
    stateGlobalPosition.altitude = stateItem.alt/1000.00;
    return stateGlobalPosition;
}

DataState::StateLocalPosition State_COMMSTOMACE::LocalPosition_COMMSTOMACE(const mace_local_position_ned_t &stateItem, const int &systemID)
{
    UNUSED(systemID);
    DataState::StateLocalPosition stateLocalPosition;
    stateLocalPosition.x = stateItem.x;
    stateLocalPosition.y = stateItem.y;
    stateLocalPosition.z = stateItem.z;
    return stateLocalPosition;
}

} //end of namespace DataCOMMS
