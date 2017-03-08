#include "mace_to_mavlink.h"

namespace DataMAVLINK {

//TODO: Kenny come back and fix these systemID components
mavlink_message_t fromAttitudeItem(std::shared_ptr<DataState::StateAttitude> attitudeItem, const uint8_t &chan, const uint8_t &compID)
{
    int systemID = 0;
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    attitude.roll = attitudeItem->roll;
    attitude.pitch = attitudeItem->pitch;
    attitude.pitch = attitudeItem->yaw;
    attitude.rollspeed = attitudeItem->rollRate;
    attitude.pitchspeed = attitudeItem->pitchRate;
    attitude.yawspeed = attitudeItem->yawRate;
    mavlink_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return(msg);
}

mavlink_message_t fromGlobalPostition(std::shared_ptr<DataState::StateGlobalPosition> positionItem, const uint8_t &chan, const uint8_t &compID)
{
    int systemID = 0;
    mavlink_message_t msg;
    mavlink_global_position_int_t position;
    position.lat = positionItem->latitude;
    position.lon = positionItem->longitude;
    position.alt = positionItem->altitude;
    mavlink_msg_global_position_int_encode_chan(systemID,compID,chan,&msg,&position);
    return(msg);
}


}
