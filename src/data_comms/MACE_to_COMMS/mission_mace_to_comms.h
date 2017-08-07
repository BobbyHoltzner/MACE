#ifndef MISSION_MACE_TO_COMMS_H
#define MISSION_MACE_TO_COMMS_H

#include <memory>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataCOMMS{

class Mission_MACETOCOMMS
{
public:

    Mission_MACETOCOMMS(const int &systemFrom, const int &systemTo, const Data::MissionKey &missionKey, const uint8_t &chan);
    Mission_MACETOCOMMS(const int &systemFrom, const uint8_t &chan);

    mace_message_t Home_MACETOCOMMS(const CommandItem::SpatialHome &missionItem);

    mace_message_t ChangeSpeed_MACETOCOMMS(const CommandItem::ActionChangeSpeed &missionItem, const uint16_t &itemIndex);

    mace_message_t Land_MACETOCOMMS(const CommandItem::SpatialLand &missionItem, const uint16_t &itemIndex);

    mace_message_t LoiterTime_MACETOCOMMS(const CommandItem::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex);

    mace_message_t LoiterTurns_MACETOCOMMS(const CommandItem::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex);

    mace_message_t LoiterUnlimited_MACETOCOMMS(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex);

    mace_message_t RTL_MACETOCOMMS(const CommandItem::SpatialRTL &missionItem, const uint16_t &itemIndex);

    mace_message_t Takeoff_MACETOCOMMS(const CommandItem::SpatialTakeoff &missionItem, const uint16_t &itemIndex);

    mace_message_t Waypoint_MACETOCOMMS(const CommandItem::SpatialWaypoint &missionItem, const uint16_t &itemIndex);

    bool MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mace_message_t &msg);


private:
    void updatePosition(const DataState::Base3DPosition &pos, mace_mission_item_t &maceItem);

private:
    void initializeMACECOMMSMissionItem(mace_mission_item_t &mavMission);
    mace_message_t packMissionItem(const mace_mission_item_t &mavMission);

private:
    int fromID;
    int toID;
    Data::MissionKey key;
    int commsChan;
};

} //end of namespace DataCOMMS

#endif // MISSION_MACE_TO_COMMS_H
