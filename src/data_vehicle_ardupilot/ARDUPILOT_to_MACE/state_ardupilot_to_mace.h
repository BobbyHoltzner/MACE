#ifndef STATE_ARDUPILOT_TO_MACE_H
#define STATE_ARDUPILOT_TO_MACE_H

#include "data_vehicle_MAVLINK/MAVLINK_to_MACE/state_mavlink_to_mace.h"

namespace DataARDUPILOT{

class State_ARDUPILOTTOMACE : public DataMAVLINK::State_MAVLINKTOMACE
{
public:
    State_ARDUPILOTTOMACE(const int &systemID);

};

} //end of namespace DataMAVLINK
#endif // STATE_ARDUPILOT_TO_MACE_H
