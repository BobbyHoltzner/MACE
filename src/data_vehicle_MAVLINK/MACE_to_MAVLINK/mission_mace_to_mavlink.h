#ifndef MISSION_MACE_TO_MAVLINK_H
#define MISSION_MACE_TO_MAVLINK_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class Mission_MACETOMAVLINK
{
public:

    Mission_MACETOMAVLINK(const int &systemID, const int &compID);

    virtual mavlink_message_t Home_MACETOMAVLINK(const MissionItem::SpatialHome &missionItem, const uint8_t &chan, const uint8_t &compID);

    virtual mavlink_message_t ChangeSpeed_MACETOMAVLINK(const MissionItem::ActionChangeSpeed &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Land_MACETOMAVLINK(const MissionItem::SpatialLand<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Land_MACETOMAVLINK(const MissionItem::SpatialLand<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterTime_MACETOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterTime_MACETOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterTurns_MACETOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterTurns_MACETOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterUnlimited_MACETOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterUnlimited_MACETOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t RTL_MACETOMAVLINK(const MissionItem::SpatialRTL &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Takeoff_MACETOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Takeoff_MACETOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Waypoint_MACETOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Waypoint_MACETOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual bool MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex, mavlink_message_t &msg);

protected:
    void initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission);
    mavlink_message_t packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan);

protected:
    int mSystemID;
    int mCompID;

};

} //end of namespace DataMAVLINK

#endif // MISSION_MACE_TO_MAVLINK_H
