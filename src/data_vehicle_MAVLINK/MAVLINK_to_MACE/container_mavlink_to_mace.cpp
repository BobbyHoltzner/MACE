#include "container_mavlink_to_mace.h"

namespace DataMAVLINK {

Container_MAVLINKTOMACE::Container_MAVLINKTOMACE(const int &systemID):
    Command_MAVLINKTOMACE(),
    Generic_MAVLINKTOMACE(systemID),
    Mission_MAVLINKTOMACE(systemID),
    State_MAVLINKTOMACE(systemID)
{

}

} //end of namepsace DataARDUPILOT
