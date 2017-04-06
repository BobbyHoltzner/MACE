#include "container_ardupilot_to_mace.h"

namespace DataARDUPILOT {

Container_ARDUPILOTTOMACE::Container_ARDUPILOTTOMACE(const int &systemID):
    Command_ARDUPILOTTOMACE(),
    Generic_ARDUPILOTTOMACE(systemID),
    Mission_ARDUPILOTTOMACE(systemID),
    State_ARDUPILOTTOMACE(systemID)
{

}

} //end of namepsace DataARDUPILOT
