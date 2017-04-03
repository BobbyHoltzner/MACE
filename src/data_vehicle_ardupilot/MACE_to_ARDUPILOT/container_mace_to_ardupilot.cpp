#include "container_mace_to_ardupilot.h"

namespace DataARDUPILOT {

Container_MACETOARDUPILOT::Container_MACETOARDUPILOT(const int &systemID, const int &compID):
    Command_MACETOARDUPILOT(systemID,compID),
    Generic_MACETOARDUPILOT(systemID,compID),
    Mission_MACETOARDUPILOT(systemID,compID),
    State_MACETOARDUPILOT(systemID,compID)
{

}

} //end of namespace DataARDUPILOT
