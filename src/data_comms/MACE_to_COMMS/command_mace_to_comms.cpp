#include "command_mace_to_comms.h"
namespace DataCOMMS{

Command_MACETOCOMMS::Command_MACETOCOMMS()
{

}

mavlink_command_long_t Command_MACETOCOMMS::initializeCommandLong()
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

mavlink_message_t Command_MACETOCOMMS::packLongMessage(const mavlink_command_long_t &cmdLong, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_command_long_t tmpItem = cmdLong;
    mavlink_msg_command_long_encode_chan(255,190,chan,&msg,&tmpItem);
    return msg;
}

mavlink_message_t Command_MACETOCOMMS::generateGetHomeMessage(const int &vehicleID, const int &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_GET_HOME_POSITION;
    cmd.target_system = vehicleID;
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mavlink_message_t Command_MACETOCOMMS::generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan)
{
    mavlink_message_t msg;
    mavlink_set_home_position_t cmd;
    cmd.latitude = vehicleHome.position.latitude * pow(10,7);
    cmd.longitude = vehicleHome.position.longitude * pow(10,7);
    cmd.altitude = vehicleHome.position.altitude * 1000;
    mavlink_msg_set_home_position_encode_chan(255,190,chan,&msg,&cmd);
    return msg;
}

mavlink_message_t Command_MACETOCOMMS::generateArmMessage(const MissionItem::ActionArm &actionArmItem, const uint8_t &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = actionArmItem.getVehicleID();
    cmd.param1 = actionArmItem.getRequestArm();
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mavlink_message_t Command_MACETOCOMMS::generateTakeoffMessage(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem, const uint8_t &chan)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.target_system = missionItem.getVehicleID();
    cmd.param5 = missionItem.position.latitude;
    cmd.param6 = missionItem.position.longitude;
    cmd.param7 = missionItem.position.altitude;
    mavlink_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

} //end of namespace DataCOMMS