#include "command_mace_to_mavlink.h"
namespace DataMAVLINK{

Command_MACETOMAVLINK::Command_MACETOMAVLINK(const int &systemID, const int &compID):
    mSystemID(systemID),mCompID(compID)
{

}

mavlink_message_t Command_MACETOMAVLINK::generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan)
{
    mavlink_message_t msg;
    mavlink_set_home_position_t cmd;
    cmd.latitude = vehicleHome.position.latitude * pow(10,7);
    cmd.longitude = vehicleHome.position.longitude * pow(10,7);
    cmd.altitude = vehicleHome.position.altitude * 1000;
    mavlink_msg_set_home_position_encode_chan(mSystemID,mCompID,chan,&msg,&cmd);
    return msg;
}

mavlink_message_t Command_MACETOMAVLINK::generateChangeMode(const int systemID, const uint8_t &chan, const int &newMode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack_chan(mSystemID,mCompID,chan,&msg,systemID,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_command_long_t Command_MACETOMAVLINK::initializeCommandLong()
{
    mavlink_command_long_t cmdLong;
    cmdLong.command = 0;
    cmdLong.confirmation = 0;
    cmdLong.param1 = 0.0;
    cmdLong.param2 = 0.0;
    cmdLong.param3 = 0.0;
    cmdLong.param4 = 0.0;
    cmdLong.param5 = 0.0;
    cmdLong.param6 = 0.0;
    cmdLong.param7 = 0.0;
    cmdLong.target_system = 0;
    cmdLong.target_component = 0;
    return cmdLong;
}

mavlink_message_t Command_MACETOMAVLINK::packLongMessage(const mavlink_command_long_t &cmdLong, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_command_long_t tmpItem = cmdLong;
    mavlink_msg_command_long_encode_chan(mSystemID,mCompID,chan,&msg,&tmpItem);
    return msg;
}

mavlink_message_t Command_MACETOMAVLINK::generateGetHomeMessage(const int &vehicleID, const int &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_GET_HOME_POSITION;
    cmd.target_system = vehicleID;
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mavlink_message_t Command_MACETOMAVLINK::generateArmMessage(const MissionItem::ActionArm &actionArmItem, const uint8_t &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = actionArmItem.getVehicleID();
    cmd.param1 = actionArmItem.getRequestArm();
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mavlink_message_t Command_MACETOMAVLINK::generateTakeoffMessage(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem, const uint8_t &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.target_system = missionItem.getVehicleID();
    cmd.param7 = missionItem.position.altitude;
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

} //end of namespace DataMAVLINK
