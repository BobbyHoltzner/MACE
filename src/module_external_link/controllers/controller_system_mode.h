#ifndef CONTROLLER_SYSTEM_MODE_H
#define CONTROLLER_SYSTEM_MODE_H

#include "helper_controller_send.h"

#include "action_send.h"
#include "action_final_receive_respond.h"
#include "action_finish.h"

namespace ExternalLink {

typedef GenericMACEController<TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,DataItem<MaceCore::ModuleCharacteristic, CommandItem::ActionChangeMode>> CONTROLLER_SYSTEMMODE_TYPE;

class ControllerSystemMode : public CONTROLLER_SYSTEMMODE_TYPE,

        public ActionSend_TargetedWithResponse<
            CONTROLLER_SYSTEMMODE_TYPE,
            MaceCore::ModuleCharacteristic,
            CommandItem::ActionChangeMode,
            mace_command_system_mode_t,
            MACE_MSG_ID_SYSTEM_MODE_ACK
        >,
        public ActionFinalReceiveRespond<
            CONTROLLER_SYSTEMMODE_TYPE,
            MaceCore::ModuleCharacteristic,
            CommandItem::ActionChangeMode,
            mace_command_system_mode_t,
            mace_system_mode_ack_t,
            MACE_MSG_ID_COMMAND_SYSTEM_MODE
        >,
        public ActionFinish<
            CONTROLLER_SYSTEMMODE_TYPE,
            MaceCore::ModuleCharacteristic,
            mace_system_mode_ack_t,
            MACE_MSG_ID_SYSTEM_MODE_ACK
        >
{


protected:


    virtual void Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, mace_command_system_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = commandItem.getTargetSystem();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        queueObj = target;

        cmd.target_system = commandItem.getTargetSystem();
        strcpy(cmd.mode, commandItem.getRequestMode().c_str());
    }


    virtual bool Construct_FinalObjectAndResponse(const mace_command_system_mode_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_system_mode_ack_t &ack, std::shared_ptr<CommandItem::ActionChangeMode> &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleObj;

        data = std::make_shared<CommandItem::ActionChangeMode>();
        data->setTargetSystem(msg.target_system);
        data->setRequestMode(std::string(msg.mode));

        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
        return true;
    }

    virtual bool Finish_Receive(const mace_system_mode_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        return true;
    }

public:

    ControllerSystemMode(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        CONTROLLER_SYSTEMMODE_TYPE(cb, queue, linkChan),
        ActionSend_TargetedWithResponse(this,
            mace_msg_command_system_mode_encode_chan),
        ActionFinalReceiveRespond(this,
                                  mace_msg_command_system_mode_decode,
                                  mace_msg_system_mode_ack_encode_chan),
        ActionFinish(this,
                     mace_msg_system_mode_ack_decode)
    {

    }

};

}

#endif // CONTROLLER_SYSTEM_MODE_H
