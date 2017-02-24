#ifndef MACE_TO_ARDUPILOT_H
#define MACE_TO_ARDUPILOT_H

#include <memory>

#include "mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataVehicleArdupilot{

    mavlink_message_t generateArmMessage(const MissionItem::ActionArm &actionArmItem, const uint8_t &chan);
    mavlink_message_t generateChangeMode(std::shared_ptr<MissionItem::ActionChangeMode> missionItem, const uint8_t &chan, const int &newMode);

    mavlink_message_t generateGetHomePosition(const int &vehicleID, const int &chan);
    mavlink_message_t generateSetHomePosition(const MissionItem::SpatialHome &vehicleHome, const int &chan);

    mavlink_message_t generateArdupilotCommandMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);
    mavlink_message_t MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex);

    mavlink_message_t MACEGuidedToArdupilot(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID);

} //end of namespace DataVehicleArdupilot
#endif // MACE_TO_ARDUPILOT_H
