#ifndef SENSOR_VERTICES_H
#define SENSOR_VERTICES_H

#include <list>
#include <string>
#include "math.h"

#include "data/i_topic_component_data_object.h"

#include "data_vehicle_generic/components.h"

#include "data/i_topic_component_data_object.h"
#include "data_vehicle_commands/abstract_mission_command.h"


namespace DataVehicleSensors
{
extern const char SensorVertices_Name[];
extern const MaceCore::TopicComponentStructure SensorVertices_Structure;

//template <class T>
class SensorVertices : public Data::NamedTopicComponentDataObject<SensorVertices_Name, &SensorVertices_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
//    SensorVertices();
    SensorVertices(const std::string &sensorName);
    std::list<DataVehicleGeneric::GlobalPosition*> getSensorVertices();
    void setSensorVertices(const std::list<DataVehicleGeneric::GlobalPosition*> &sensorVertices);
    void insertSensorVertice(DataVehicleGeneric::GlobalPosition *verticePosition);

private:
    std::string sensorName;
    DataVehicleGeneric::PositionFrame positionFrame;
    std::list<DataVehicleGeneric::GlobalPosition*> verticeLocations;

};

}
#endif // SENSOR_VERTICES_H
