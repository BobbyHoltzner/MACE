#ifndef MISSION_MACE_TO_ARDUPILOT_H
#define MISSION_MACE_TO_ARDUPILOT_H

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/mission_mace_to_mavlink.h"

namespace DataARDUPILOT{

class Mission_MACETOARDUPILOT : public DataMAVLINK::Mission_MACETOMAVLINK
{
public:

    Mission_MACETOARDUPILOT(const int &systemID, const int &compID);

};

} //end of namespace DataMAVLINK

#endif // MISSION_MACE_TO_ARDUPILOT_H
