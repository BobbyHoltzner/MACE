#include "vehicle_object_ardupilot.h"

namespace DataARDUPILOT {

VehicleObject_ARDUPILOT::VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp):
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_ARDUPILOTTOMACE(vehicleID),Container_MACETOARDUPILOT(systemID,systemComp)
{
    data = new DataContainer_ARDUPILOT();
    parser = new ARDUPILOTParser(data);
}

}//end of namespace DataARDUPILOT
