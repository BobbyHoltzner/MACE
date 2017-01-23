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
        m_Armed = armed;
    }

    bool getVehicleArmed(){
        return(m_Armed);
    }

public:
    bool operator == (const VehicleOperatingStatus &rhs) {
        if(this->m_Armed != rhs.m_Armed){
            return false;
        }
        return true;
    }

    bool operator != (const VehicleOperatingStatus &rhs) {
        return !(*this == rhs);
    }

private:

    bool m_Armed;

};

}

#endif // VEHICLE_OPERATING_STATUS_H
