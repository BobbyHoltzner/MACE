#ifndef SENSOR_VERTICES_H
#define SENSOR_VERTICES_H

#include <list>
#include <string>
#include "math.h"

#include "data/i_topic_component_data_object.h"

#include "data/positional_coordinate_frame.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"


namespace DataVehicleSensors
{
extern const char SensorVerticesGlobal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure;

class SensorVertices_Global : public Data::NamedTopicComponentDataObject<SensorVerticesGlobal_Name, &SensorVerticesGlobal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Global();
    SensorVertices_Global(const std::string &sensorName);

public:
    std::vector<DataState::StateGlobalPosition> getSensorVertices() const;
    void setSensorVertices(const std::vector<DataState::StateGlobalPosition> &verticeVector);

private:
    std::string sensorName;
    Data::PositionalFrame positionFrame;
    std::vector<DataState::StateGlobalPosition> verticeLocations;
};


extern const char SensorVerticesLocal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure;

class SensorVertices_Local : public Data::NamedTopicComponentDataObject<SensorVerticesLocal_Name, &SensorVerticesLocal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Local();
    SensorVertices_Local(const std::string &sensorName);

public:
    std::vector<DataState::StateLocalPosition> getSensorVertices() const;
    void setSensorVertices(const std::vector<DataState::StateLocalPosition> &verticeVector);

private:
    std::string sensorName;
    Data::PositionalFrame positionFrame;
    std::vector<DataState::StateLocalPosition> verticeLocations;
};

}
#endif // SENSOR_VERTICES_H
