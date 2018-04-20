#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

namespace MAVLINKVehicleControllers {

struct MAVLINKModeStruct
{
    uint8_t targetID;
    uint8_t vehicleMode;
};

using SystemModeSend = Controllers::ActionSend<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MAVLINKModeStruct>,
    MaceCore::ModuleCharacteristic,
    MAVLINKModeStruct,
    mavlink_set_mode_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

using SystemModeFinish = Controllers::ActionFinish<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MAVLINKModeStruct>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

class ControllerSystemMode : public Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MAVLINKModeStruct>,
        public SystemModeSend,
        public SystemModeFinish
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void Construct_Send(const MAVLINKModeStruct &commandMode, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_set_mode_t &mode, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        mode.target_system = commandMode.targetID;
        mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode.custom_mode = commandMode.vehicleMode;

    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    ControllerSystemMode(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MAVLINKModeStruct>(cb, queue, linkChan),
        SystemModeSend(this, mavlink_msg_set_mode_encode_chan),
        SystemModeFinish(this, mavlink_msg_command_ack_decode)
    {

    }


};

} //end of namespace MAVLINKVehicleControllers

#endif // MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
