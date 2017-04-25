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
protected:
    //mutable std::mutex ardupilot_stateTopicMutex;

public:
    Data::DataGetSetNotifier<DataARDUPILOT::VehicleFlightMode> ArducopterFlightMode;

public:

    /*
    void setArdupilotFlightMode(const DataARDUPILOT::VehicleFlightMode &info)
    {
        std::lock_guard<std::mutex> guard(ardupilot_stateTopicMutex);
        m_ArducopterFlightMode = info;
    }

    DataARDUPILOT::VehicleFlightMode getArdupilotFlightMode() const{
        std::lock_guard<std::mutex> guard(ardupilot_stateTopicMutex);
        return  m_ArducopterFlightMode.get();
    }
    */

public:


};

#endif // DATA_CONTAINER_ARDUPILOT_H
