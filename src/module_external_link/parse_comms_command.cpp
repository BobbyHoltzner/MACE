#include "module_external_link.h"
//Ken: The target system here in this file is not correct.

void ModuleExternalLink::ParseCommsCommand(const mace_command_short_t *message)
{
    return;

    switch(static_cast<CommandItem::COMMANDITEM>(message->command))
    {
    case(CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND):
    {
        BroadcastLogicToAllVehicles(message->target_system, [this, message](int vehicleID){
            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
            mace_message_t msg;
            mace_msg_command_ack_encode_chan(vehicleID, 0, m_LinkChan, &msg, &commandACK);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

            CommandItem::ActionMissionCommand missionCommand;
            missionCommand.setTargetSystem(vehicleID);
            missionCommand.setMissionCommandType(static_cast<Data::MissionCommandAction>(message->param));
            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
                ptr->Event_IssueMissionCommand(this, missionCommand);
            });
        });
        break;
    }
    case(CommandItem::COMMANDITEM::CI_ACT_ARM):
    {
        BroadcastLogicToAllVehicles(message->target_system, [this, message](int vehicleID){
            CommandItem::ActionArm tmpArm;
            tmpArm.setTargetSystem(vehicleID);
            tmpArm.setVehicleArm(fabs(message->param) <= 0.001 ? false : true);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
            mace_message_t msg;
            mace_msg_command_ack_encode_chan(vehicleID, 0, m_LinkChan, &msg, &commandACK);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

            //notify core
            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
                ptr->Event_IssueCommandSystemArm(this, tmpArm);
            });
        });
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH):
    {
        BroadcastLogicToAllVehicles(message->target_system, [this, message](int vehicleID){
            CommandItem::SpatialRTL tmpRTL;
            tmpRTL.setTargetSystem(vehicleID);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
            mace_message_t msg;
            mace_msg_command_ack_encode_chan(vehicleID, 0, m_LinkChan,&msg,&commandACK);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
                ptr->Event_IssueCommandRTL(this, tmpRTL);
            });
        });
        break;
    }
    default:
        break;
    }
}

void ModuleExternalLink::ParseCommsCommand(const mace_command_long_t *message)
{
    return;

    switch(message->command)
    {
    case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF):
    {
        BroadcastLogicToAllVehicles(message->target_system, [this, message](int vehicleID){

            CommandItem::SpatialTakeoff tmpTakeoff;
            tmpTakeoff.setTargetSystem(vehicleID);
            if(message->param1 > 0.0)
            {
                tmpTakeoff.position->setX(message->param5);
                tmpTakeoff.position->setY(message->param6);
            }
            tmpTakeoff.position->setZ(message->param7);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
            mace_message_t msg;
            mace_msg_command_ack_encode_chan(vehicleID,0,m_LinkChan,&msg,&commandACK);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
                ptr->Event_IssueCommandTakeoff(this, tmpTakeoff);
            });
        });
        break;
    }
    case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND):
    {
        BroadcastLogicToAllVehicles(message->target_system, [this, message](int vehicleID){
            CommandItem::SpatialLand tmpLand;
            tmpLand.setTargetSystem(vehicleID);
            tmpLand.position->setX(message->param5);
            tmpLand.position->setY(message->param6);
            tmpLand.position->setZ(message->param7);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
            mace_message_t msg;
            mace_msg_command_ack_encode_chan(vehicleID,0,m_LinkChan,&msg,&commandACK);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
                ptr->Event_IssueCommandLand(this, tmpLand);
            });
        });
        break;
    }
    case(MAV_CMD_DO_CHANGE_SPEED):
    {
        break;
    }
    default:
        break;
    }
}

void ModuleExternalLink::ParseCommsCommand(const mace_command_system_mode_t *message)
{  
    return;
    CommandItem::ActionChangeMode tmpMode;
    tmpMode.setTargetSystem(message->target_system);
    tmpMode.setRequestMode(std::string(message->mode));
    std::cout<<"We are trying to change the mode RX in external link: "<<tmpMode.getRequestMode()<<std::endl;

    //acknowledge receiving the command
    mace_system_mode_ack_t modeACK;
    modeACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
    mace_message_t msg;
    mace_msg_system_mode_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&modeACK);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
        ptr->Event_ChangeSystemMode(this, tmpMode);
    });
}
