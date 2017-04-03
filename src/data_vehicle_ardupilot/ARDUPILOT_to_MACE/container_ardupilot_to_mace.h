#ifndef CONTAINER_ARDUPILOT_TO_MACE_H
#define CONTAINER_ARDUPILOT_TO_MACE_H

#include "ARDUPILOT_to_MACE/command_ardupilot_to_mace.h"
#include "ARDUPILOT_to_MACE/generic_ardupilot_to_mace.h"
#include "ARDUPILOT_to_MACE/mission_ardupilot_to_mace.h"
#include "ARDUPILOT_to_MACE/state_ardupilot_to_mace.h"

namespace DataARDUPILOT {

class Container_ARDUPILOTTOMACE : public Command_ARDUPILOTTOMACE, public Generic_ARDUPILOTTOMACE, public Mission_ARDUPILOTTOMACE, public State_ARDUPILOTTOMACE
{
public:
    Container_ARDUPILOTTOMACE(const int &systemID);
};

} //end of namespace DataARDUPILOT

#endif // CONTAINER_ARDUPILOT_TO_MACE_H
