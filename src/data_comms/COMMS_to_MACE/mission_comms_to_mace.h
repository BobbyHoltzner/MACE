#ifndef MISSION_COMMS_TO_MACE_H
#define MISSION_COMMS_TO_MACE_H

#include <iostream>
#include <memory>

#include "mace.h"
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

namespace DataCOMMS{

class Mission_COMMSTOMACE
{
public:
    Mission_COMMSTOMACE();
    std::shared_ptr<CommandItem::AbstractCommandItem> Covert_COMMSTOMACE(const mace_mission_item_t &maceItem);

    void Home_COMMSTOMACE(const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem);

    void ChangeSpeed_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem);

    void LoiterTime_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time &missionItem);

    void LoiterTurns_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns &missionItem);

    void LoiterUnlimited_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited &missionItem);

    void RTL_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem);

    void Takeoff_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff &missionItem);

    void Waypoint_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint &missionItem);

private:
    void updatePosition(const mace_mission_item_t &maceItem, DataState::Base3DPosition &pos);

};

} //end of namespace DataCOMMS
#endif // MISSION_COMMS_TO_MACE_H
