#ifndef DATA_CONTAINER_ARDUPILOT_H
#define DATA_CONTAINER_ARDUPILOT_H

#include "data_vehicle_MAVLINK/data_container_mavlink.h"

class DataContainer_ARDUPILOT : public DataMAVLINK::DataContainer_MAVLINK
{
public:
    DataContainer_ARDUPILOT();
};

#endif // DATA_CONTAINER_ARDUPILOT_H
