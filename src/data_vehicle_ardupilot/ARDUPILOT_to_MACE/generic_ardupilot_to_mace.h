#ifndef GENERIC_ARDUPILOT_TO_MACE_H
#define GENERIC_ARDUPILOT_TO_MACE_H

#include "data_vehicle_MAVLINK/MAVLINK_to_MACE/generic_mavlink_to_mace.h"


namespace DataARDUPILOT{

class Generic_ARDUPILOTTOMACE : public DataMAVLINK::Generic_MAVLINKTOMACE
{
public:
    Generic_ARDUPILOTTOMACE(const int &systemID);

};

} //end of namespace DataMAVLINK

#endif // GENERIC_ARDUPILOT_TO_MACE_H
