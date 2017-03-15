#include "mace_to_ardupilot.h"

namespace DataArdupilot {

mavlink_message_t generateChangeMode(const int systemID, const uint8_t &chan, const int &newMode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,systemID,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_message_t generateArmMessage(const MissionItem::ActionArm &actionArmItem, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmdLong;

    cmdLong.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmdLong.confirmation = 0;
    cmdLong.param1 = actionArmItem.getRequestArm();
    cmdLong.param2 = 0.0;
    cmdLong.param3 = 0.0;
    cmdLong.param4 = 0.0;
    cmdLong.param5 = 0.0;
    cmdLong.param6 = 0.0;
    cmdLong.param7 = 0.0;
    cmdLong.target_system = actionArmItem.getVehicleID();
    cmdLong.target_component = 0;

    mavlink_msg_command_long_encode_chan(255,190,chan,&msg,&cmdLong);
    //mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleID,0,command,0,armFlag,0,0,0,0,0,0);
    return msg;
}

mavlink_message_t generateGetHomePosition(const int &vehicleID, const int &chan)
{
    uint16_t command = MAV_CMD_GET_HOME_POSITION;
    mavlink_message_t msg;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,vehicleID,0,command,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
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

mavlink_message_t generateTakeoffMessage(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem, const uint8_t &chan, const uint8_t &compID)
{
    //This is command number 22
    uint16_t command = MAV_CMD_NAV_TAKEOFF;
    mavlink_message_t msg;
    int systemID = missionItem.getVehicleID();
    float latitude = missionItem.position.latitude;
    float longitude = missionItem.position.longitude;
    float altitude = missionItem.position.altitude;
    mavlink_msg_command_long_pack_chan(255,190,chan,&msg,systemID,compID,command,0,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
    return msg;
}

} //end of namespace DataVehicleArdupilot
