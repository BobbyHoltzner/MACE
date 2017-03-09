#ifndef MACE_TO_MAVLINK_H
#define MACE_TO_MAVLINK_H

#include "mavlink.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

mavlink_message_t fromFlightModeItem(std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> flightModeItem, const int &systemID, const uint8_t &chan, const uint8_t &compID);

mavlink_message_t fromAttitudeItem(std::shared_ptr<DataState::StateAttitude> attitudeItem, const uint8_t &chan, const uint8_t &compID);

mavlink_message_t fromGlobalPostition(std::shared_ptr<DataState::StateGlobalPosition> positionItem, const uint8_t &chan, const uint8_t &compID);


} //end of namespace DataVehicleArdupilot


#endif // MACE_TO_MAVLINK_H
