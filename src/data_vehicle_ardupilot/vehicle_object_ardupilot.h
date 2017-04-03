#ifndef VEHICLE_OBJECT_ARDUPILOT_H
#define VEHICLE_OBJECT_ARDUPILOT_H

#include "ARDUPILOT_to_MACE/container_ardupilot_to_mace.h"
#include "MACE_to_ARDUPILOT/container_mace_to_ardupilot.h"
#include "data_container_ardupilot.h"
#include "ardupilot_parser.h"

namespace DataARDUPILOT {

class VehicleObject_ARDUPILOT : public Container_ARDUPILOTTOMACE, public Container_MACETOARDUPILOT
{
public:
    VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp);

public:
    ARDUPILOTParser *parser;
private:
    DataContainer_ARDUPILOT *data;
    int m_VehicleID;
    int m_SystemID;
    int m_SystemComp;
};

} //end of namespace DataARDUPILOT

#endif // VEHICLE_OBJECT_ARDUPILOT_H
