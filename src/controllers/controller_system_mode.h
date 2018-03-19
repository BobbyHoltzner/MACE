#ifndef CONTROLLER_SYSTEM_MODE_H
#define CONTROLLER_SYSTEM_MODE_H

#include "generic_controller.h"
#include "generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "actions/action_send.h"
#include "actions/action_final_receive_respond.h"
#include "actions/action_finish.h"

namespace Controllers {


template <typename MESSAGETYPE>
using SystemModeSend = ActionSend<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;

template <typename MESSAGETYPE>
using SystemMode_FinalReceiveRespond = ActionFinalReceiveRespond<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    mace_system_mode_ack_t,
    MACE_MSG_ID_COMMAND_SYSTEM_MODE
>;

template <typename MESSAGETYPE>
using SystemModeFinish = ActionFinish<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_system_mode_ack_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;


template<typename MESSAGETYPE>
class ControllerSystemMode : public GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,

        public SystemModeSend<MESSAGETYPE>,
        public SystemMode_FinalReceiveRespond<MESSAGETYPE>,
        public SystemModeFinish<MESSAGETYPE>
{




protected:


    virtual void Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_system_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
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

    virtual bool Finish_Receive(const mace_system_mode_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    ControllerSystemMode(const IMessageNotifier<MESSAGETYPE>* cb, MessageModuleTransmissionQueue<MESSAGETYPE> * queue, int linkChan) :
        GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>(cb, queue, linkChan),
        SystemModeSend<MESSAGETYPE>(this, mace_msg_command_system_mode_encode_chan),
        SystemMode_FinalReceiveRespond<MESSAGETYPE>(this, mace_msg_command_system_mode_decode, mace_msg_system_mode_ack_encode_chan),
        SystemModeFinish<MESSAGETYPE>(this, mace_msg_system_mode_ack_decode)
    {

    }

};

}

#endif // CONTROLLER_SYSTEM_MODE_H
