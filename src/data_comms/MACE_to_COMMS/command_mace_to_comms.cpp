#include "command_mace_to_comms.h"
namespace DataCOMMS{

Command_MACETOCOMMS::Command_MACETOCOMMS()
{

}

mace_command_long_t Command_MACETOCOMMS::initializeCommandLong()
{
    mace_command_long_t cmdLong;
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

mace_message_t Command_MACETOCOMMS::packLongMessage(const mace_command_long_t &cmdLong, const uint8_t &chan)
{
    mace_message_t msg;
    mace_command_long_t tmpItem = cmdLong;
    mace_msg_command_long_encode_chan(255,190,chan,&msg,&tmpItem);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateGetHomeMessage(const int &vehicleID, const int &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_GET_HOME_POSITION;
    cmd.target_system = vehicleID;
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateSetHomePosition(const CommandItem::SpatialHome &vehicleHome, const int &chan)
{
    mace_message_t msg;
    mace_set_home_position_t cmd;
    cmd.latitude = vehicleHome.position.latitude * pow(10,7);
    cmd.longitude = vehicleHome.position.longitude * pow(10,7);
    cmd.altitude = vehicleHome.position.altitude * 1000;
    mace_msg_set_home_position_encode_chan(255,190,chan,&msg,&cmd);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateArmMessage(const CommandItem::ActionArm &actionArmItem, const uint8_t &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)Data::CommandItemType::CI_ACT_ARM;
    cmd.target_system = actionArmItem.getTargetSystem();
    cmd.param1 = actionArmItem.getRequestArm();
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateTakeoffMessage(const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)Data::CommandItemType::CI_NAV_TAKEOFF;
    cmd.target_system = missionItem.getTargetSystem();
    cmd.param1 = (missionItem.getPositionFlag())? 1.0 : 0.0;
    cmd.param5 = missionItem.position.latitude;
    cmd.param6 = missionItem.position.longitude;
    cmd.param7 = missionItem.position.altitude;
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateLandMessage(const CommandItem::SpatialLand<DataState::StateGlobalPosition> &command, const uint8_t &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)Data::CommandItemType::CI_NAV_LAND;
    cmd.target_system = command.getTargetSystem();
    cmd.param1 = (command.getLandFlag())? 1.0 : 0.0;
    cmd.param5 = command.position.latitude;
    cmd.param6 = command.position.longitude;
    cmd.param7 = command.position.altitude;
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateRTLMessage(const CommandItem::SpatialRTL &command, const uint8_t &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    cmd.command = (uint8_t)Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH;
    cmd.target_system = command.getTargetSystem();
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

mace_message_t Command_MACETOCOMMS::generateMissionCommandMessage(const CommandItem::ActionMissionCommand &command, const uint8_t &chan)
{
    mace_command_long_t cmd = initializeCommandLong();
    mace_message_t msg = packLongMessage(cmd,chan);
    return msg;
}

} //end of namespace DataCOMMS
