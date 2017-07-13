#ifndef HELPER_MISSION_MAVLINK_TO_MACE_H
#define HELPER_MISSION_MAVLINK_TO_MACE_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK {

class Helper_MissionMAVLINKtoMACE
{
public:
    Helper_MissionMAVLINKtoMACE(const int &originatingID);

    ~Helper_MissionMAVLINKtoMACE();

    std::shared_ptr<CommandItem::AbstractCommandItem> Convert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem);

    void convertHome(const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem);

    void convertChangespeed(const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem);

    void convertLand(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand &missionItem);

    void convertLoiterTime(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time &missionItem);

    void convertLoiterTurns(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns &missionItem);

    void convertLoiterUnlimted(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited &missionItem);

    void convertRTL(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem);

    void convertTakeoff(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff &missionItem);

    void convertWaypoint(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint &missionItem);

    void updatePosition(const mavlink_mission_item_t &mavlinkItem, DataState::Base3DPosition &pos);

private:
    int systemID;
};

} //end of namespace DataMAVLINK

#endif // HELPER_MISSION_MAVLINK_TO_MACE_H
