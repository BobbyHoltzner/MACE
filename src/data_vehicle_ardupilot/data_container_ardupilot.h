#ifndef DATA_CONTAINER_ARDUPILOT_H
#define DATA_CONTAINER_ARDUPILOT_H

#include "data_vehicle_MAVLINK/data_container_mavlink.h"
#include "data_vehicle_ardupilot/components/vehicle_flightMode.h"

#include "data/data_get_set_notifier.h"

class DataContainer_ARDUPILOT : public DataMAVLINK::DataContainer_MAVLINK
{
public:
    DataContainer_ARDUPILOT();


    ///////////////////////////////////////////////////////////////////////////////
    /// ARDUPILOT STATE ITEMS
    //////////////////////////////////////////////////////////////////////////////
public:
    Data::DataGetSetNotifier<DataARDUPILOT::VehicleFlightMode> ArdupilotFlightMode;

};

#endif // DATA_CONTAINER_ARDUPILOT_H
