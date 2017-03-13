#ifndef STATE_MACE_TO_COMMS_H
#define STATE_MACE_TO_COMMS_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataCOMMS{

class State_COMMSTOMAVLINK
{
public:
    virtual mavlink_message_t Attitude_COMMSTOMAVLINK(const DataState::StateAttitude &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    virtual mavlink_message_t GlobalPosition_COMMSTOMAVLINK(const DataState::StateGlobalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);

    virtual mavlink_message_t LocalPosition_COMMSTOMAVLINK(const DataState::StateLocalPosition &stateItem, const int &systemID, const uint8_t &compID, const uint8_t &chan);
};

} //end of namespace DataCOMMS

#endif // STATE_MACE_TO_COMMS_H
