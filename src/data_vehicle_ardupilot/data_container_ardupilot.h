#ifndef DATA_CONTAINER_ARDUPILOT_H
#define DATA_CONTAINER_ARDUPILOT_H

#include "data_vehicle_MAVLINK/data_container_mavlink.h"
#include "data_vehicle_ardupilot/components/vehicle_flightMode.h"

class DataContainer_ARDUPILOT : public DataMAVLINK::DataContainer_MAVLINK
{
public:
    DataContainer_ARDUPILOT();

public:
    std::shared_ptr<DataARDUPILOT::VehicleFlightMode> m_ArducopterFlightMode;

};

#endif // DATA_CONTAINER_ARDUPILOT_H
