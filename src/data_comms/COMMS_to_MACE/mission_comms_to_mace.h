#ifndef MISSION_COMMS_TO_MACE_H
#define MISSION_COMMS_TO_MACE_H

#include <iostream>
#include <memory>

#include "mace.h"
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

namespace DataCOMMS{

class Mission_COMMSTOMACE
{
public:
    Mission_COMMSTOMACE();
    std::shared_ptr<MissionItem::AbstractMissionItem> Covert_COMMSTOMACE(const mace_mission_item_t &maceItem);

    void Home_COMMSTOMACE(const int &vehicleID, const mace_set_home_position_t &maceItem, MissionItem::SpatialHome &missionItem);

    void ChangeSpeed_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::ActionChangeSpeed &missionItem);

    void LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem);
    void LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem);

    void LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem);
    void LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem);

    void LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem);
    void LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem);

    void RTL_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialRTL &missionItem);

    void Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem);
    void Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem);

    void Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem);
    void Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem);
};

} //end of namespace DataCOMMS
#endif // MISSION_COMMS_TO_MACE_H
