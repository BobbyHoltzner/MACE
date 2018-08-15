#ifndef CONTROLLER_GUIDED_TARGET_ITEM_LOCAL_H
#define CONTROLLER_GUIDED_TARGET_ITEM_LOCAL_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/target_items/dynamic_target_list.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"
#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

using namespace mace::pose;

namespace MAVLINKVehicleControllers {

struct TargetControllerStructLocal
{
    uint8_t targetID;
    TargetItem::CartesianDynamicTarget target;
};

template <typename T>
using GuidedTGTLocalBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MavlinkEntityKey, T>,
    T,
    mavlink_set_position_target_local_ned_t
>;

template <typename T>
using GuidedTGTLocalSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MavlinkEntityKey, T>,
    MavlinkEntityKey,
    T,
    mavlink_set_position_target_local_ned_t,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED
>;

template <typename T>
using GuidedTGTLocalFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MavlinkEntityKey, T>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_position_target_local_ned_t,
    MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
>;

template <typename TARGETITEM>
class ControllerGuidedTargetItem_Local : public Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MavlinkEntityKey, TARGETITEM>,
        public GuidedTGTLocalBroadcast<TARGETITEM>,
        public GuidedTGTLocalSend<TARGETITEM>,
        public GuidedTGTLocalFinish<TARGETITEM>
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

protected:


    virtual void Construct_Broadcast(const TARGETITEM &commandItem, const MavlinkEntityKey &sender, std::vector<mavlink_set_position_target_local_ned_t> &vec)
    {
        UNUSED(sender);

        mavlink_set_position_target_local_ned_t targetItem;
        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.targetID;
        targetItem.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        vec.push_back(targetItem);

        FillTargetItem(commandItem,targetItem);
    }

    virtual bool Construct_Send(const TARGETITEM &commandItem, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_position_target_local_ned_t &targetItem, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetModuleFromMAVLINKVehicleID(commandItem.targetID);

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.targetID;
        targetItem.target_component = 0;

        FillTargetItem(commandItem,targetItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_position_target_local_ned_t &msg, const MavlinkEntityKey &sender, uint8_t& ack, MavlinkEntityKey &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = 0;
        return true;
    }

protected:
    void FillTargetItem(const TargetControllerStructLocal &targetItem, mavlink_set_position_target_local_ned_t &mavlinkItem);

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
    ControllerGuidedTargetItem_Local(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue<mavlink_message_t, MavlinkEntityKey> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MavlinkEntityKey, TARGETITEM>(cb, queue, linkChan),
        GuidedTGTLocalBroadcast<TARGETITEM>(this, mavlink_msg_set_position_target_local_ned_encode_chan),
        GuidedTGTLocalSend<TARGETITEM>(this, mavlink_msg_set_position_target_local_ned_encode_chan),
        GuidedTGTLocalFinish<TARGETITEM>(this, mavlink_msg_position_target_local_ned_decode)
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_LOCAL_H
