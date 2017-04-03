#include "command_mace_to_ardupilot.h"
namespace DataARDUPILOT{

Command_MACETOARDUPILOT::Command_MACETOARDUPILOT(const int &systemID, const int &compID):
    DataMAVLINK::Command_MACETOMAVLINK(systemID,compID)
{

}

} //end of namespace DataARDUPILOT
