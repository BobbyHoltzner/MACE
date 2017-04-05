#include "data_container_mavlink.h"

namespace DataMAVLINK{

DataContainer_MAVLINK::DataContainer_MAVLINK():
    m_CurrentVehicleState(NULL), m_CurrentVehicleFuel(NULL), m_CurrentVehicleGPS(NULL),
    m_CurrentVehicleText(NULL), m_CurrentGlobalPosition(NULL), m_CurrentGlobalPositionEx(NULL),
    m_CurrentLocalPosition(NULL), m_CurrentVehicleAttitude(NULL)
{

}


} //end of namespace DataMAVLINK
