#ifndef CONTAINER_MACE_TO_ARDUPILOT_H
#define CONTAINER_MACE_TO_ARDUPILOT_H

#include "data_vehicle_ardupilot/MACE_to_ARDUPILOT/command_mace_to_ardupilot.h"
#include "data_vehicle_ardupilot/MACE_to_ARDUPILOT/generic_mace_to_ardupilot.h"
#include "data_vehicle_ardupilot/MACE_to_ARDUPILOT/mission_mace_to_ardupilot.h"
#include "data_vehicle_ardupilot/MACE_to_ARDUPILOT/state_mace_to_ardupilot.h"

namespace DataARDUPILOT {

class Container_MACETOARDUPILOT : public Command_MACETOARDUPILOT, public Generic_MACETOARDUPILOT, public Mission_MACETOARDUPILOT, public State_MACETOARDUPILOT
{
public:
    Container_MACETOARDUPILOT(const int &systemID, const int &compID);
};

} //end of namespace DataARDUPILOT
#endif // CONTAINER_MACE_TO_ARDUPILOT_H
