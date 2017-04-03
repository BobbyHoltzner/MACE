#include "container_mace_to_mavlink.h"

namespace DataMAVLINK {

Container_MACETOMAVLINK::Container_MACETOMAVLINK(const int &systemID, const int &compID):
    Command_MACETOMAVLINK(systemID,compID),
    Generic_MACETOMAVLINK(systemID,compID),
    Mission_MACETOMAVLINK(systemID,compID),
    State_MACETOMAVLINK(systemID,compID)
{

}

} //end of namespace DataARDUPILOT
