#include "vehicle_object_mavlink.h"

namespace DataMAVLINK{

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &systemID, const int &systemComp) :
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_MAVLINKTOMACE(vehicleID),Container_MACETOMAVLINK(systemID,systemComp)
{
    data = new DataContainer_MAVLINK();
    parser = new MAVLINKParser(data);
}

} //end of namespace DataMAVLINK
