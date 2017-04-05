#ifndef GENERIC_MACE_TO_ARDUPILOT_H
#define GENERIC_MACE_TO_ARDUPILOT_H

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/generic_mace_to_mavlink.h"

namespace DataARDUPILOT{

class Generic_MACETOARDUPILOT : public DataMAVLINK::Generic_MACETOMAVLINK
{
public:

    Generic_MACETOARDUPILOT(const int &systemID, const int &compID);

};

} //end of namespace DataMAVLINK

#endif // GENERIC_MACE_TO_ARDUPILOT_H
