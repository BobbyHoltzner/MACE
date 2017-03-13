#ifndef MISSION_MACE_TO_COMMS_H
#define MISSION_MACE_TO_COMMS_H

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

namespace DataCOMMS{

class Mission_COMMSTOMAVLINK
{
public:
    virtual mavlink_message_t Home_COMMSTOMAVLINK(const MissionItem::SpatialHome &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Land_COMMSTOMAVLINK(const MissionItem::SpatialLand<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Land_COMMSTOMAVLINK(const MissionItem::SpatialLand<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterTime_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterTime_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterTurns_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterTurns_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t LoiterUnlimited_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t LoiterUnlimited_COMMSTOMAVLINK(const MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t RTL_COMMSTOMAVLINK(const MissionItem::SpatialRTL &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Takeoff_COMMSTOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Takeoff_COMMSTOMAVLINK(const MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t Waypoint_COMMSTOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    virtual mavlink_message_t Waypoint_COMMSTOMAVLINK(const MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    virtual mavlink_message_t MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
private:
    void initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission);
    mavlink_message_t packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan);
};

} //end of namespace DataCOMMS

#endif // MISSION_MACE_TO_COMMS_H
