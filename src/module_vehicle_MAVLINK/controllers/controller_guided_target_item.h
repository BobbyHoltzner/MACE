#ifndef CONTROLLER_GUIDED_TARGET_ITEM_H
#define CONTROLLER_GUIDED_TARGET_ITEM_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/target_items/dynamic_target_list.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

namespace MAVLINKVehicleControllers {

struct TargetControllerStruct
{
    uint8_t targetID;
    TargetItem::DynamicTargetList::DynamicTarget target;
};
template <typename T>
using GuidedTGTSend = Controllers::ActionSend<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_set_position_target_local_ned_t,
    MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
>;

template <typename T>
using GuidedTGTFinish = Controllers::ActionFinish<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_position_target_local_ned_t,
    MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
>;

template <typename TARGETITEM>
class ControllerGuidedTargetItem : public Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, TARGETITEM>,
        public GuidedTGTSend<TARGETITEM>,
        public GuidedTGTFinish<TARGETITEM>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const TARGETITEM &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_set_position_target_local_ned_t &targetItem, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj.ID = commandItem.targetID;
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.targetID;
        targetItem.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillTargetItem(commandItem,targetItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_position_target_local_ned_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t& ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.x; //this is MAV_COMMAND_RESULT
        return true;
    }

protected:
    void FillTargetItem(const TargetControllerStruct &targetItem, mavlink_set_position_target_local_ned_t &mavlinkItem);

    mavlink_set_position_target_local_ned_t initializeMAVLINKTargetItem()
    {
        mavlink_set_position_target_local_ned_t targetItem;
        targetItem.afx = 0.0;
        targetItem.afy = 0.0;
        targetItem.afz = 0.0;
        targetItem.coordinate_frame = MAV_FRAME_LOCAL_NED;
        targetItem.target_component =0;
        targetItem.target_system = 0;
        targetItem.time_boot_ms = 0;
        targetItem.type_mask = 65535; //by default we want to ignore all of the values
        targetItem.vx = 0.0;
        targetItem.vy = 0.0;
        targetItem.vz = 0.0;
        targetItem.x = 0.0;
        targetItem.y = 0.0;
        targetItem.yaw_rate = 0.0;
        targetItem.z = 0.0;

        return targetItem;
    }

public:
    ControllerGuidedTargetItem(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, TARGETITEM>(cb, queue, linkChan),
        GuidedTGTSend<TARGETITEM>(this, mavlink_msg_set_position_target_local_ned_encode_chan),
        GuidedTGTFinish<TARGETITEM>(this, mavlink_msg_position_target_local_ned_decode)
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_H
