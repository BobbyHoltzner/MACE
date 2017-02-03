#ifndef SENSOR_VERTICES_H
#define SENSOR_VERTICES_H

#include <list>
#include <string>
#include "math.h"

#include "data/i_topic_component_data_object.h"

#include "data_vehicle_generic/components.h"

#include "data/i_topic_component_data_object.h"
#include "data_vehicle_commands/abstract_mission_item.h"

#include "data/positional_coordinate_frame.h"

namespace DataVehicleSensors
{
extern const char SensorVerticesLocal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure;

extern const char SensorVerticesGlobal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure;

template <class T>
class SensorVerticesBase
{
public:
    std::list<T*> getSensorVertices();
    void setSensorVertices(const std::list<T*> verticeList);
    void insertSensorVertice(T *verticePosition);

protected:
    std::string sensorName;
    Data::PositionalFrame positionFrame;
    std::list<T*> verticeLocations;
};

class SensorVertices_Global : public SensorVerticesBase<DataVehicleGeneric::GlobalPosition>, public Data::NamedTopicComponentDataObject<SensorVerticesGlobal_Name, &SensorVerticesGlobal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Global();
    SensorVertices_Global(const std::string &sensorName);

};

class SensorVertices_Local : public SensorVerticesBase<DataVehicleGeneric::LocalPosition>, public Data::NamedTopicComponentDataObject<SensorVerticesLocal_Name, &SensorVerticesLocal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Local();
    SensorVertices_Local(const std::string &sensorName);
};

}
#endif // SENSOR_VERTICES_H
