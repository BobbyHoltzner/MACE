#ifndef COMMAND_ARDUPILOT_TO_MACE_H
#define COMMAND_ARDUPILOT_TO_MACE_H

#include "data_vehicle_MAVLINK/MAVLINK_to_MACE/command_mavlink_to_mace.h"

namespace DataARDUPILOT{

class Command_ARDUPILOTTOMACE : public DataMAVLINK::Command_MAVLINKTOMACE
{
public:
    Command_ARDUPILOTTOMACE();
};

} //end of namespace DataMAVLINK

#endif // COMMAND_ARDUPILOT_TO_MACE_H
