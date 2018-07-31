#ifndef CONTROLLER_GUIDED_MISSION_ITEM_H
#define CONTROLLER_GUIDED_MISSION_ITEM_H

#include "common/common.h"

#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/spatial_items/spatial_waypoint.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

namespace MAVLINKVehicleControllers {

template <typename T>
using GuidedMISend = Controllers::ActionSend<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_mission_item_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;

template <typename T>
using GuidedMIFinish = Controllers::ActionFinish<
    mavlink_message_t,
    Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;

template <typename MISSIONITEM>
class ControllerGuidedMissionItem : public Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MISSIONITEM>,
        public GuidedMISend<MISSIONITEM>,
        public GuidedMIFinish<MISSIONITEM>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const MISSIONITEM &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_item_t &missionItem, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj.ID = commandItem.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        missionItem = initializeMAVLINKMissionItem();
        missionItem.target_system = commandItem.getTargetSystem();
        missionItem.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillMissionItem(commandItem,missionItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_mission_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.type; //this is MAV_MISSION_RESULT
        return true;
    }

protected:
    void FillMissionItem(const CommandItem::SpatialWaypoint &commandItem, mavlink_mission_item_t &mavlinkItem);

    mavlink_mission_item_t initializeMAVLINKMissionItem()
    {
        mavlink_mission_item_t missionItem;
        missionItem.autocontinue = 1;
        missionItem.command = 0;
        missionItem.current = 2;
        missionItem.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        missionItem.param1 = 0.0;
        missionItem.param2 = 0.0;
        missionItem.param3 = 0.0;
        missionItem.param4 = 0.0;
        missionItem.seq = 0;
        missionItem.target_system = 0;
        missionItem.target_component = 0;
        missionItem.x = 0.0;
        missionItem.y = 0.0;
        missionItem.z = 0.0;
        //missionItem.mission_type = MAV_MISSION_TYPE_AUTO;

        return missionItem;
    }

public:
    ControllerGuidedMissionItem(const Controllers::IMessageNotifier<mavlink_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, int linkChan) :
        Controllers::GenericControllerQueueDataWithModule<mavlink_message_t, MISSIONITEM>(cb, queue, linkChan),
        GuidedMISend<MISSIONITEM>(this, mavlink_msg_mission_item_encode_chan),
        GuidedMIFinish<MISSIONITEM>(this, mavlink_msg_mission_ack_decode)
    {

    }

    virtual ~ControllerGuidedMissionItem() = default;

};

} //end of namespace MAVLINKVehicleControllers

#endif // CONTROLLER_GUIDED_MISSION_ITEM_H
