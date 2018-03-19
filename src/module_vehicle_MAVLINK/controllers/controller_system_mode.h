#ifndef CONTROLLER_SYSTEM_MODE_H
#define CONTROLLER_SYSTEM_MODE_H

#include "common/common.h"

#include "generic_controller.h"
#include "generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

namespace MAVLINKControllers {


template <typename T>
using SystemModeSend = ActionSend<
    GenericControllerQueueDataWithModule<T, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

template<typename MESSAGETYPE>
class ControllerSystemMode : public GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
        public SystemModeSend<MESSAGETYPE>
{

protected:

    virtual void Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, mavlink_set_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);

        cmd.target_system = commandItem.getTargetSystem();
        cmd.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.custom_mode = newMode;
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
        SystemModeSend<MESSAGETYPE>(this, mavlink_msg_set_mode_encode_chan)
    {

    }

};

}

#endif // CONTROLLER_SYSTEM_MODE_H
