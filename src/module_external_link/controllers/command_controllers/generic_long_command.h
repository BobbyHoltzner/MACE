#ifndef BASE_LONG_COMMAND_H
#define BASE_LONG_COMMAND_H

#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "mace_core/module_characteristics.h"

#include "../generic_mace_controller.h"


#include "../helper_controller_send.h"

namespace ExternalLink {

template <typename T>
using CONTROLLER_TYPE = GenericMACEController<TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,DataItem<MaceCore::ModuleCharacteristic, T>>;


template <typename T>
using HelperControllerSetLONGCOMMAND = HelperControllerSend<
CONTROLLER_TYPE<T>,
T,
mace_command_long_t,
mace_command_ack_t,
MACE_MSG_ID_COMMAND_LONG,
MACE_MSG_ID_COMMAND_ACK,
MaceCore::ModuleCharacteristic,
T
>;



template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericLongCommand : public CONTROLLER_TYPE<COMMANDDATASTRUCTURE>,
        public HelperControllerSetLONGCOMMAND<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_long_t &) const = 0;

    virtual void BuildCommand(const mace_command_long_t &, std::shared_ptr<COMMANDDATASTRUCTURE>) const= 0;


protected:

    virtual void BuildMessage_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mace_command_long_t &cmd, MaceCore::ModuleCharacteristic &queueObj, MaceCore::ModuleCharacteristic &target)
    {
        target.ID = data.getTargetSystem();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        queueObj = target;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);
    }

    virtual bool BuildData_Send(const mace_command_long_t &msg, std::shared_ptr<COMMANDDATASTRUCTURE> data, mace_command_ack_t &ack, MaceCore::ModuleCharacteristic &vehicleFrom, MaceCore::ModuleCharacteristic &queueObj)
    {
        vehicleFrom.ID = msg.target_system;
        vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleFrom;

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        this-> template BuildCommand(msg, data);

        ack.command = COMMANDTYPE;
        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        return true;
    }

public:

    Controller_GenericLongCommand(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        CONTROLLER_TYPE<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        HelperControllerSetLONGCOMMAND<COMMANDDATASTRUCTURE>(this,
            [](uint8_t system_id, uint8_t compID, uint8_t chan, mace_message_t* msg, const mace_command_long_t* data){ mace_msg_command_long_encode_chan(system_id, compID, chan, msg, data); },
            [](const mace_message_t* msg, mace_command_long_t* data){mace_msg_command_long_decode(msg, data);},
            [](uint8_t system_id, uint8_t compID, uint8_t chan, mace_message_t* msg, const mace_command_ack_t* data){mace_msg_command_ack_encode_chan(system_id, compID, chan, msg, data); }
        )
    {

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




/*
namespace ExternalLink {

template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class GenericLongCommandController : public GenericMACEController<
        TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,
        DataItem<MaceCore::ModuleCharacteristic, AbstractCommandItem>
        >
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_long_t &) const = 0;

    virtual void BuildCommand(const mace_command_long_t *, std::shared_ptr<COMMANDDATASTRUCTURE>) const= 0;
public:


    GenericLongCommandController(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        GenericMACEController(cb, queue, linkChan)
    {

        AddTriggeredLogic<MACE_MSG_ID_COMMAND_LONG, mace_command_long_t >( mace_msg_command_long_decode,
                [this](const mace_command_long_t  &msg, const MaceCore::ModuleCharacteristic &sender){
                    this->ParseCommsCommand(&msg, sender);
                }
        );
    }

    void IssueCommand(const COMMANDDATASTRUCTURE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        if(mLog)
        {
            std::stringstream buffer;
            buffer << commandItem;

            mLog->debug("Command Controller is requesting the system to takeoff.");
            mLog->info(buffer.str());
        }

        OptionalParameter<MaceCore::ModuleCharacteristic> target = OptionalParameter<MaceCore::ModuleCharacteristic>();
        if(commandItem.getTargetSystem() != 0)
        {
            MaceCore::ModuleCharacteristic module;
            module.ID = commandItem.getTargetSystem();
            module.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            target = module;
        }

        mace_command_long_t cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(commandItem, cmd);

        // TODO - MTB- NEED TO ITERATE OVER ALL ATTACHED VEHICLES
        QueueCommand(cmd, mace_msg_command_long_encode_chan, sender, target);
    }

private:

    template <typename T, typename FUNC>
    void QueueCommand(const T &cmd, FUNC func, const MaceCore::ModuleCharacteristic &sender, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) {

        if(target.IsSet())
        {
            QueueTransmission(target(), MACE_MSG_ID_COMMAND_ACK, [this, func, cmd, sender, target](){
                EncodeMessage(func, cmd, sender, target);
            });
        }
        else {
            EncodeMessage(func, cmd, sender, target);
        }

    }


    void ParseCommsCommand(const mace_command_long_t* message, const MaceCore::ModuleCharacteristic &sender)
    {
        switch(message->command)
        {
        case(COMMANDTYPE):
        {
            MaceCore::ModuleCharacteristic target = sender;

            MaceCore::ModuleCharacteristic vehicleFrom;
            vehicleFrom.ID = message->target_system;
            vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            std::shared_ptr<COMMANDDATASTRUCTURE> tmpTakeoff = std::make_shared<COMMANDDATASTRUCTURE>();
            BuildCommand(message, tmpTakeoff);

            onDataReceived(sender, tmpTakeoff);

            //acknowledge receiving the command
            mace_command_ack_t commandACK;
            commandACK.command = COMMANDTYPE;
            commandACK.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

            //switch up target/sender because sending ACK back.
            EncodeMessage(mace_msg_command_ack_encode_chan, commandACK, vehicleFrom, target);

            break;
        }
        default:
            break;
        }
    }

    void ParseCommsCommand(const mace_command_short_t* message)
    {

    }



};


}

*/

#endif // BASE_LONG_COMMAND_H
