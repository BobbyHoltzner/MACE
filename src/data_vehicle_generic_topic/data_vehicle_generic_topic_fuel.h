#ifndef DATA_VEHICLE_GENERIC_TOPIC_FUEL_H
#define DATA_VEHICLE_GENERIC_TOPIC_FUEL_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleGenericTopic {

extern const char GenericVehicleTopicFuel_name[];
extern const MaceCore::TopicComponentStructure GenericVehicleTopicFuel_structure;

class DataVehicleGenericTopic_Fuel : public Data::NamedTopicComponentDataObject<GenericVehicleTopicFuel_name, &GenericVehicleTopicFuel_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    void setBatteryVoltage(const double &voltage){
        this->voltage = voltage;
    }
    double getBatteryVoltage(){
        return voltage;
    }

    void setBatteryCurrent(const double &current){
        this->current = current;
    }
    double getBatteryCurrent(){
        return current;
    }

    void setBatteryRemaining(const double &batteryRemaing){
        this->batteryRemaing = batteryRemaing;
    }
    double getBatteryRemaining(){
        return batteryRemaing;
    }

private:
    double voltage;
    double current;
    double batteryRemaing;
};

} //end of namespace DataVehicleGenericTopic

#endif // DATA_VEHICLE_GENERIC_TOPIC_FUEL_H
