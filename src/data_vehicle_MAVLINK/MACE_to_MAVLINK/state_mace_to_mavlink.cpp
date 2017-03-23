#include "state_mace_to_mavlink.h"

namespace DataMAVLINK {

mavlink_message_t State_MACETOMAVLINK::Attitude_MACETOMAVLINK(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    attitude.roll = stateItem.roll;
    attitude.pitch = stateItem.pitch;
    attitude.pitch = stateItem.yaw;
    attitude.rollspeed = stateItem.rollRate;
    attitude.pitchspeed = stateItem.pitchRate;
    attitude.yawspeed = stateItem.yawRate;
    mavlink_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return(msg);
}

mavlink_message_t State_MACETOMAVLINK::GlobalPosition_MACETOMAVLINK(const DataState::StateGlobalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_global_position_int_t position;
    position.lat = (int32_t)stateItem.latitude * pow(10,7);
    position.lon = (int32_t)stateItem.longitude * pow(10,7);
    position.alt = (int32_t)stateItem.altitude * 1000.0;
    mavlink_msg_global_position_int_encode_chan(systemID,compID,chan,&msg,&position);
    return(msg);
}

mavlink_message_t State_MACETOMAVLINK::LocalPosition_MACETOMAVLINK(const DataState::StateLocalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_local_position_ned_t position;
    position.x = stateItem.x;
    position.y = stateItem.y;
    position.z = stateItem.z;
    mavlink_msg_local_position_ned_encode_chan(systemID,compID,chan,&msg,&position);
    return(msg);
}

} //end of namespace DataMAVLINK
