#include "mace_to_ardupilot.h"

namespace DataArdupilot {

mavlink_message_t generateChangeMode(const int systemID, const uint8_t &chan, const int &newMode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,systemID,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_message_t generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan)
{
    mavlink_message_t msg;
    int32_t latitude = vehicleHome.position.latitude * pow(10,7);
    int32_t longitude = vehicleHome.position.longitude * pow(10,7);
    int32_t altitude = vehicleHome.position.altitude * 1000;
    float qVector;
    mavlink_msg_set_home_position_pack_chan(255,190,chan,&msg,vehicleHome.getVehicleID(),latitude,longitude,altitude,0.0,0.0,0.0,&qVector,0.0,0.0,0.0);
    return msg;
}

} //end of namespace DataVehicleArdupilot
