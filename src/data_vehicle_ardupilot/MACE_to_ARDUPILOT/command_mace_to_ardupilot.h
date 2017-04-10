#ifndef COMMAND_MACE_TO_ARDUPILOT_H
#define COMMAND_MACE_TO_ARDUPILOT_H

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/command_mace_to_mavlink.h"

namespace DataARDUPILOT{

class Command_MACETOARDUPILOT : public DataMAVLINK::Command_MACETOMAVLINK
{
public:
    Command_MACETOARDUPILOT(const int &systemID, const int &compID);
};


} //end of namepsace DataMAVLINK
#endif // COMMAND_MACE_TO_ARDUPILOT_H
