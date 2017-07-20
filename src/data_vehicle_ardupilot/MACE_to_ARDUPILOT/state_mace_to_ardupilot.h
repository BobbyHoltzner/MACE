#ifndef STATE_MACE_TO_ARDUPILOT_H
#define STATE_MACE_TO_ARDUPILOT_H

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/state_mace_to_mavlink.h"

namespace DataARDUPILOT{

class State_MACETOARDUPILOT : public DataMAVLINK::State_MACETOMAVLINK
{
public:

    State_MACETOARDUPILOT(const int &systemID, const int &compID);

};

} //end of namespace DataMAVLINK

#endif // STATE_MACE_TO_ARDUPILOT_H
