#ifndef VEHICLE_OPERATING_ATTITUDE_H
#define VEHICLE_OPERATING_ATTITUDE_H

#include "mavlink.h"

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_topics/attitude_topic.h"

namespace DataVehicleArdupilot
{
class VehicleOperatingAttitude : public DataStateTopic::AttitudeTopic
{
public:
    //declare some gets here
    void handleMSG(const mavlink_attitude_t &msg);
};

}


#endif // VEHICLE_OPERATING_ATTITUDE_H
