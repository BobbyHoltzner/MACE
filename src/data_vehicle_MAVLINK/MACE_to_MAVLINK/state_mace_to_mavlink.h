#ifndef STATE_MACE_TO_MAVLINK_H
#define STATE_MACE_TO_MAVLINK_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK{

class State_MACETOMAVLINK
{
public:

    State_MACETOMAVLINK(const int &systemID, const int &compID);

    virtual mavlink_message_t Attitude_MACETOMAVLINK(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    virtual mavlink_message_t GlobalPosition_MACETOMAVLINK(const DataState::StateGlobalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    virtual mavlink_message_t LocalPosition_MACETOMAVLINK(const DataState::StateLocalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

protected:
    int mSystemID;
    int mCompID;
};

} //end of namespace DataMAVLINK

#endif // STATE_MACE_TO_MAVLINK_H
