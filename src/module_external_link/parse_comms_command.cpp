#include "module_external_link.h"

void ModuleExternalLink::ParseCommsCommand(const mace_command_short_t *message)
{
    switch(static_cast<CommandItem::COMMANDITEM>(message->command))
    {
    case(CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND):
    {
        //acknowledge receiving the command
        mace_command_ack_t commandACK;
        commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND;
        commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
        mace_message_t msg;
        mace_msg_command_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&commandACK);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        CommandItem::ActionMissionCommand missionCommand;
        missionCommand.setTargetSystem(message->target_system);
        missionCommand.setMissionCommandType(static_cast<Data::MissionCommandAction>(message->param));
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueMissionCommand(this, missionCommand);
        });
        break;
    }
    case(CommandItem::COMMANDITEM::CI_ACT_ARM):
    {
        CommandItem::ActionArm tmpArm;
        tmpArm.setTargetSystem(message->target_system);
        tmpArm.setVehicleArm(fabs(message->param) <= 0.001 ? false : true);

        //acknowledge receiving the command
        mace_command_ack_t commandACK;
        commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM;
        commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
        mace_message_t msg;
        mace_msg_command_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&commandACK);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        //notify core
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandSystemArm(this, tmpArm);
        });
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH):
    {
        CommandItem::SpatialRTL tmpRTL;
        tmpRTL.setTargetSystem(message->target_system);

        //acknowledge receiving the command
        mace_command_ack_t commandACK;
        commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
        commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
        mace_message_t msg;
        mace_msg_command_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&commandACK);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandRTL(this, tmpRTL);
        });
        break;
    }
    default:
        break;
    }
}

void ModuleExternalLink::ParseCommsCommand(const mace_command_long_t *message)
{
    switch(message->command)
    {
    case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF):
    {
        CommandItem::SpatialTakeoff tmpTakeoff;
        tmpTakeoff.setTargetSystem(message->target_system);
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
        mace_msg_command_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&commandACK);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandTakeoff(this, tmpTakeoff);
        });
        break;
    }
    case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND):
    {
        CommandItem::SpatialLand tmpLand;
        tmpLand.setTargetSystem(message->target_system);
        tmpLand.position->setX(message->param5);
        tmpLand.position->setY(message->param6);
        tmpLand.position->setZ(message->param7);

        //acknowledge receiving the command
        mace_command_ack_t commandACK;
        commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND;
        commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
        mace_message_t msg;
        mace_msg_command_ack_encode_chan(message->target_system,0,m_LinkChan,&msg,&commandACK);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->Event_IssueCommandLand(this, tmpLand);
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
    CommandItem::ActionChangeMode cmd;
    cmd.setTargetSystem(message->target_system);
    cmd.setRequestMode(std::string(message->mode));
}
