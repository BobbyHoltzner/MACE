#ifndef MISSION_PARSER_VEHICLE_ARDUPILOT_H
#define MISSION_PARSER_VEHICLE_ARDUPILOT_H

#include <mavlink.h>
#include <memory>

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"


class MissionParserArdupilot
{
public:
    static mavlink_message_t generateMissionMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const int &itemIndex, const int &compID, const uint8_t &chan);
};

#endif // MISSION_PARSER_VEHICLE_ARDUPILOT_H
