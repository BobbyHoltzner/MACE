#ifndef HELPER_MISSION_COMMS_TO_MACE_H
#define HELPER_MISSION_COMMS_TO_MACE_H

#include <memory>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataInterface_MACE {

class Helper_MissionCOMMStoMACE
{
public:
    Helper_MissionCOMMStoMACE();

    ~Helper_MissionCOMMStoMACE();

    void updateIDS(const int &originatingID);

    std::shared_ptr<CommandItem::AbstractCommandItem> Convert_COMMSTOMACE(const mace_mission_item_t &maceItem);

    void convertHome(const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem);

    void convertChangespeed(const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem);

    void convertLand(const mace_mission_item_t &maceItem, CommandItem::SpatialLand &missionItem);

    void convertLoiterTime(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time &missionItem);

    void convertLoiterTurns(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns &missionItem);

    void convertLoiterUnlimted(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited &missionItem);

    void convertRTL(const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem);

    void convertTakeoff(const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff &missionItem);

    void convertWaypoint(const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint &missionItem);

    DataState::Base3DPosition getBasePosition(const mace_mission_item_t &maceItem);

    void updatePosition(const mace_mission_item_t &maceItem, DataState::Base3DPosition &pos);

private:
    int systemID;
};

} //end of namespace DataInterface_MACE

#endif // HELPER_MISSION_COMMS_TO_MACE_H
