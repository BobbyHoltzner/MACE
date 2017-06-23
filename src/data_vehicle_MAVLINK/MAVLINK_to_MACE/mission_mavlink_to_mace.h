#ifndef MISSION_MAVLINK_TO_MACE_H
#define MISSION_MAVLINK_TO_MACE_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class Mission_MAVLINKTOMACE
{
public:
    Mission_MAVLINKTOMACE(const int &systemID);

    std::shared_ptr<CommandItem::AbstractCommandItem> Covert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem);
    void Home_MAVLINKTOMACE(const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem);

    void ChangeSpeed_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem);

    void Land_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand<DataState::StateGlobalPosition> &missionItem);

    CommandItem::SpatialLoiter_Time LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem);
    void LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem);

    void LoiterTurns_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem);
    void LoiterTurns_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem);

    void LoiterUnlimited_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem);
    void LoiterUnlimited_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem);

    void RTL_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem);

    void Takeoff_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem);
    void Takeoff_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem);

    void Waypoint_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem);
    void Waypoint_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem);

protected:
    int mSystemID;
};

} //end of namespace DataMAVLINK
#endif // MISSION_MAVLINK_TO_MACE_H
