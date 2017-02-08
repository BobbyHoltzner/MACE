#ifndef ARDUPILOT_TO_MACE_MISSION_H
#define ARDUPILOT_TO_MACE_MISSION_H

#include <memory>

#include "mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataVehicleArdupilot
{

class ArdupilotToMACEMission
{
public:

    static std::shared_ptr<MissionItem::AbstractMissionItem> MAVLINKMissionToMACEMission(const mavlink_mission_item_t &missionItem);
};

}
#endif // ARDUPILOT_TO_MACE_MISSION_H
