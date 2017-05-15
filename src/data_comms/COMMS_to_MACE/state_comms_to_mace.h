#ifndef STATE_COMMS_TO_MACE_H
#define STATE_COMMS_TO_MACE_H

#include <math.h>

#include "mace.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

namespace DataCOMMS{

class State_COMMSTOMACE
{
public:
    State_COMMSTOMACE();
    static void testAnotherFunction();
    static DataState::StateAttitude Attitude_COMMSTOMACE(const mace_attitude_t &stateItem, const int &systemID);
    static DataState::StateAttitude AttitudeRates_COMMSTOMACE(const mace_attitude_rates_t &stateItem, const int &systemID);
    static DataState::StateGlobalPosition GlobalPosition_COMMSTOMACE(const mace_global_position_int_t &stateItem, const int &systemID);
    static DataState::StateLocalPosition LocalPosition_COMMSTOMACE(const mace_local_position_ned_t &stateItem, const int &systemID);
};

} //end of namespace DataCOMMS
#endif // STATE_COMMS_TO_MACE_H
