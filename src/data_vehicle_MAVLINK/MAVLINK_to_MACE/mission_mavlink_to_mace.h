#ifndef MISSION_MAVLINK_TO_MACE_H
#define MISSION_MAVLINK_TO_MACE_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/positional_coordinate_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class Mission_MAVLINKTOMACE
{
public:
    Mission_MAVLINKTOMACE();

    void ChangeSpeed_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::ActionChangeSpeed &missionItem);

    void LoiterTime_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem);
    void LoiterTime_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem);

    void LoiterTurns_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem);
    void LoiterTurns_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem);

    void LoiterUnlimited_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem);
    void LoiterUnlimited_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem);

    void RTL_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialRTL &missionItem);

    void Takeoff_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem);
    void Takeoff_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem);

    void Waypoint_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem);
    void Waypoint_MACETOMAVLINK(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem);
};

} //end of namespace DataMAVLINK
#endif // MISSION_MAVLINK_TO_MACE_H
