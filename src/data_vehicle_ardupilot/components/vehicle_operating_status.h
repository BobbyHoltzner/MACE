#ifndef VEHICLE_OPERATING_STATUS_H
#define VEHICLE_OPERATING_STATUS_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleArdupilot
{

extern const char VehicleOperatingStatus_name[];
extern const MaceCore::TopicComponentStructure VehicleOperatingStatus_structure;

class VehicleOperatingStatus : public Data::NamedTopicComponentDataObject<VehicleOperatingStatus_name, &VehicleOperatingStatus_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setVehicleArmed(const bool armed){
        vehicleArmed = armed;
    }

    bool getVehicleArmed(){
        return(vehicleArmed);
    }

public:
    bool operator == (const VehicleOperatingStatus &rhs) {
        if(this->vehicleArmed != rhs.vehicleArmed){
            return false;
        }
        return true;
    }

    bool operator != (const VehicleOperatingStatus &rhs) {
        return !(*this == rhs);
    }

private:

    bool vehicleArmed;

};

}

#endif // VEHICLE_OPERATING_STATUS_H
