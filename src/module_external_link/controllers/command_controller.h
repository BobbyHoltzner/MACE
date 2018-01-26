#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H
#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

#include "mace_core/module_characteristics.h"

#include "generic_mace_controller.h"


namespace ExternalLink {

class CommandController : public GenericMACEController<
        TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,
        DataItem<MaceCore::ModuleCharacteristic, std::shared_ptr<AbstractCommandItem>>
        >
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

public:
    CommandController(const MACEControllerInterface* cb, int linkChan) :
        GenericMACEController(cb, linkChan)
    {
        /*
        case MACE_MSG_ID_COMMAND_LONG:
        {
            mace_command_long_t decodedMSG;
            mace_msg_command_long_decode(message,&decodedMSG);
            this->ParseCommsCommand(&decodedMSG);
            break;
        }
        case MACE_MSG_ID_COMMAND_SHORT:
        {
            mace_command_short_t decodedMSG;
            mace_msg_command_short_decode(message,&decodedMSG);
            this->ParseCommsCommand(&decodedMSG);
            break;
        }
        case MACE_MSG_ID_COMMAND_SYSTEM_MODE:
        {
            mace_command_system_mode_t decodedMSG;
            mace_msg_command_system_mode_decode(message,&decodedMSG);
            this->ParseCommsCommand(&decodedMSG);
            break;
        }
            */

        AddTriggeredLogic<MACE_MSG_ID_COMMAND_LONG, mace_command_long_t >( mace_msg_command_long_decode,
                [this](const mace_command_long_t  &msg, const MaceCore::ModuleCharacteristic &sender){
                    this->ParseCommsCommand(&msg, sender);
                }
        );

        AddTriggeredLogic<MACE_MSG_ID_COMMAND_SYSTEM_MODE, mace_command_system_mode_t >( mace_msg_command_system_mode_decode,
                [this](const mace_command_system_mode_t  &msg, const MaceCore::ModuleCharacteristic &sender){
                    HandleSystemModeCommand(msg, sender);
                }
        );
    }

    void HandleSystemModeCommand(const mace_command_system_mode_t  &msg, const MaceCore::ModuleCharacteristic &sender) {
        MaceCore::ModuleCharacteristic target = sender;

        MaceCore::ModuleCharacteristic vehicleFrom;
        vehicleFrom.ID = msg.target_system;
        vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::shared_ptr<CommandItem::ActionChangeMode> tmpMode = std::make_shared<CommandItem::ActionChangeMode>();
        tmpMode->setTargetSystem(msg.target_system);
        tmpMode->setRequestMode(std::string(msg.mode));

        onDataReceived(sender, tmpMode);

        std::cout<<"We are trying to change the mode RX in external link: "<< tmpMode->getRequestMode()<<std::endl;

        //acknowledge receiving the command
        mace_system_mode_ack_t modeACK;
        modeACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        EncodeMessage(mace_msg_system_mode_ack_encode_chan, modeACK, vehicleFrom, target);
    }


    void setSystemArm(const CommandItem::ActionArm &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(mLog)
        {
            std::stringstream buffer;
            buffer << commandItem;

            mLog->debug("Command Controller is requesting the system to arm.");
            mLog->info(buffer.str());
        }

        mace_command_short_t cmd = initializeCommandShort();
        cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;
        cmd.param = commandItem.getRequestArm();

        QueueCommand(cmd, mace_msg_command_short_encode_chan, sender, target);
    }


    void setSystemTakeoff(const CommandItem::SpatialTakeoff &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        if(mLog)
        {
            std::stringstream buffer;
            buffer << commandItem;

            mLog->debug("Command Controller is requesting the system to takeoff.");
            mLog->info(buffer.str());
        }

        MaceCore::ModuleCharacteristic target;
        target.ID = commandItem.getTargetSystem();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        mace_command_long_t cmd = initializeCommandLong();
        cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;
        Data::CoordinateFrameType cf = commandItem.position->getCoordinateFrame();

        if(commandItem.position->has2DPositionSet())
        {
            cmd.param1 = 1.0;
            cmd.param5 = commandItem.position->getX();
            cmd.param6 = commandItem.position->getY();
        }
        cmd.param7 = commandItem.position->getZ();

        QueueCommand(cmd, mace_msg_command_long_encode_chan, sender, target);
    }


    void setSystemLand(const CommandItem::SpatialLand &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(mLog)
        {
            std::stringstream buffer;
            buffer << commandItem;

            mLog->debug("Command Controller is requesting the system to land.");
            mLog->info(buffer.str());
        }

        mace_command_long_t cmd = initializeCommandLong();
        cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
        {
            cmd.param5 = commandItem.position->getX() * pow(10,7);
            cmd.param6 = commandItem.position->getY() * pow(10,7);
            cmd.param7 = commandItem.position->getZ() * 1000;
        }

        QueueCommand(cmd, mace_msg_command_long_encode_chan, sender, target);
    }


    void setSystemRTL(const CommandItem::SpatialRTL &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(mLog)
            mLog->debug("Command Controller is requesting the system to RTL.");

        mace_command_short_t cmd = initializeCommandShort();
        cmd.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        QueueCommand(cmd, mace_msg_command_short_encode_chan, sender, target);
    }


    void setSystemMode(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        if(mLog)
            mLog->debug("Command Controller is requesting the system to change modes.");

        mace_command_system_mode_t cmd;
        cmd.target_system = commandItem.getTargetSystem();
        strcpy(cmd.mode,commandItem.getRequestMode().c_str());

        MaceCore::ModuleCharacteristic target;
        target.ID = commandItem.getTargetSystem();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        QueueCommand(cmd, mace_msg_command_system_mode_encode_chan, sender, target);

        QueueTransmission(target, MACE_MSG_ID_SYSTEM_MODE_ACK, [this, cmd, sender, target](){
            EncodeMessage(mace_msg_command_system_mode_encode_chan, cmd, sender, target);
        });
    }


    void setSystemMissionCommand(const CommandItem::ActionMissionCommand &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(mLog)
            mLog->debug("Command Controller is requesting to set the system mission command.");

        mace_command_short_t cmd = initializeCommandShort();
        cmd.command = (uint16_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;
        cmd.param = (uint8_t)commandItem.getMissionCommandAction();

        QueueCommand(cmd, mace_msg_command_short_encode_chan, sender, target);
    }


private:

    template <typename T, typename FUNC>
    void QueueCommand(const T &cmd, FUNC func, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target) {

        QueueTransmission(target, MACE_MSG_ID_COMMAND_ACK, [this, func, cmd, sender, target](){
            EncodeMessage(func, cmd, sender, target);
        });

    }


    void ParseCommsCommand(const mace_command_long_t* message, const MaceCore::ModuleCharacteristic &sender)
    {
        switch(message->command)
        {
        case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF):
        {
            MaceCore::ModuleCharacteristic target = sender;

            MaceCore::ModuleCharacteristic vehicleFrom;
            vehicleFrom.ID = message->target_system;
            vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            std::shared_ptr<CommandItem::SpatialTakeoff> tmpTakeoff = std::make_shared<CommandItem::SpatialTakeoff>();
            if(message->param1 > 0.0)
            {
                tmpTakeoff->position->setX(message->param5);
                tmpTakeoff->position->setY(message->param6);
            }
            tmpTakeoff->setTargetSystem(message->target_system);
            tmpTakeoff->position->setZ(message->param7);

            onDataReceived(sender, tmpTakeoff);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

            //switch up target/sender because sending ACK back.
            EncodeMessage(mace_msg_command_ack_encode_chan, commandACK, vehicleFrom, target);

            break;
        }
        case((uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND):
        {
            MaceCore::ModuleCharacteristic target = sender;

            MaceCore::ModuleCharacteristic vehicleFrom;
            vehicleFrom.ID = message->target_system;
            vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            std::shared_ptr<CommandItem::SpatialLand> tmpLand = std::make_shared<CommandItem::SpatialLand>();
            tmpLand->setTargetSystem(message->target_system);
            tmpLand->position->setX(message->param5);
            tmpLand->position->setY(message->param6);
            tmpLand->position->setZ(message->param7);

            onDataReceived(sender, tmpLand);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

            EncodeMessage(mace_msg_command_ack_encode_chan, commandACK, vehicleFrom, target);

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

    void ParseCommsCommand(const mace_command_short_t* message)
    {

    }

    void ParseCommsCommand(const mace_command_system_mode_t* message)
    {

    }


    mace_command_short_t initializeCommandShort()
    {
        mace_command_short_t cmdShort;
        cmdShort.command = 0;
        cmdShort.confirmation = 0;
        cmdShort.param = 0.0;
        cmdShort.target_system = 0;
        cmdShort.target_component = 0;
        return cmdShort;
    }


    mace_command_long_t initializeCommandLong()
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
};

}
#endif // COMMAND_CONTROLLER_H
