#ifndef MISSION_ARDUPILOT_TO_MACE_H
#define MISSION_ARDUPILOT_TO_MACE_H

#include "data_vehicle_MAVLINK/MAVLINK_to_MACE/mission_mavlink_to_mace.h"

namespace DataARDUPILOT{

class Mission_ARDUPILOTTOMACE : public DataMAVLINK::Mission_MAVLINKTOMACE
{
public:
    Mission_ARDUPILOTTOMACE(const int &systemID);

};

} //end of namespace DataMAVLINK
#endif // MISSION_ARDUPILOT_TO_MACE_H
