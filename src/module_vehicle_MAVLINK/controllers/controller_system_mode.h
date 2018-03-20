#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

namespace MAVLINKVehicleControllers {


template <typename MESSAGETYPE>
using SystemModeSend = Controllers::ActionSend<
    MESSAGETYPE,
    Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mavlink_set_mode_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;

template<typename MESSAGETYPE>
class ControllerSystemMode : public Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
        public SystemModeSend<MESSAGETYPE>
{

protected:

    virtual void Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_set_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);

        cmd.target_system = commandItem.getTargetSystem();
        cmd.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        //KEN FIX THIS MODE IS BROKEN
        //cmd.custom_mode = newMode;
    }

    /*
     * Add back in with ActionFinish
    virtual bool Finish_Receive(const mace_system_mode_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }
    */

public:

    ControllerSystemMode(const Controllers::IMessageNotifier<MESSAGETYPE>* cb, Controllers::MessageModuleTransmissionQueue<MESSAGETYPE> * queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>(cb, queue, linkChan),
        SystemModeSend<MESSAGETYPE>(this, mavlink_msg_set_mode_encode_chan)
    {

    }

};

}

#endif // MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
