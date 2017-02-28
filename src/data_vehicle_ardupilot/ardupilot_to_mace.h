#ifndef ARDUPILOT_TO_MACE_H
#define ARDUPILOT_TO_MACE_H

#include <memory>

#include "mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataArdupilot{

std::shared_ptr<MissionItem::AbstractMissionItem> MAVLINKMissionToMACEMission(const int &vehicleID, const mavlink_mission_item_t &missionItem);

} //end of namespace DataVehicleArdupilot
#endif // ARDUPILOT_TO_MACE_H
