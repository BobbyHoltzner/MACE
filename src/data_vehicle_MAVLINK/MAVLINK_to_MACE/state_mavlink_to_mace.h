#ifndef STATE_MAVLINK_TO_MACE_H
#define STATE_MAVLINK_TO_MACE_H

#include <math.h>

#include "mavlink.h"

#include "data/coordinate_frame.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

namespace DataMAVLINK{

class State_MAVLINKTOMACE
{
public:
    State_MAVLINKTOMACE(const int &systemID);

    virtual DataState::StateAttitude Attitude_MAVLINKTOMACE(const mavlink_attitude_t &stateItem);
    virtual DataState::StateGlobalPosition GlobalPosition_MAVLINKTOMACE(const mavlink_global_position_int_t &stateItem);
    virtual DataState::StateLocalPosition LocalPosition_MAVLINKTOMACE(const mavlink_local_position_ned_t &stateItem);

protected:
    int mSystemID;
};

} //end of namespace DataMAVLINK
#endif // STATE_MAVLINK_TO_MACE_H
