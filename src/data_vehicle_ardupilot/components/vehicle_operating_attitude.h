#ifndef VEHICLE_OPERATING_ATTITUDE_H
#define VEHICLE_OPERATING_ATTITUDE_H

#include "mavlink.h"

#include "data/i_topic_component_data_object.h"
#include "data_vehicle_generic/attitude.h"

namespace DataVehicleArdupilot
{
class VehicleOperatingAttitude : public DataVehicleGeneric::Attitude
{
public:
    //declare some gets here
    void handleMSG(const mavlink_attitude_t &msg);
};

}


#endif // VEHICLE_OPERATING_ATTITUDE_H
