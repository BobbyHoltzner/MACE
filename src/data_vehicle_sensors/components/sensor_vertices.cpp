#include "sensor_vertices.h"

namespace DataVehicleSensors
{
const char SensorVerticesGlobal_Name[] = "SensorVerticesGlobal";
const char SensorVerticesLocal_Name[] = "SensorVerticesLocal";

const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<Data::PositionalFrame>("PositionFrame");
    structure.AddTerminal<std::list<DataState::LocalPosition*>>("SensorVertices");
    return structure;
}();

const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<Data::PositionalFrame>("PositionFrame");
    structure.AddTerminal<std::list<DataState::GlobalPosition*>>("SensorVertices");
    return structure;
}();


MaceCore::TopicDatagram SensorVertices_Global::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<Data::PositionalFrame>("PositionFrame",positionFrame);
    datagram.AddTerminal<std::list<DataState::GlobalPosition*>>("SensorVertices",verticeLocations);
    return datagram;
}

MaceCore::TopicDatagram SensorVertices_Local::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<Data::PositionalFrame>("PositionFrame",positionFrame);
    datagram.AddTerminal<std::list<DataState::LocalPosition*>>("SensorVertices",verticeLocations);
    return datagram;
}

void SensorVertices_Global::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    positionFrame = datagram.GetTerminal<Data::PositionalFrame>("PositionFrame");
    verticeLocations = datagram.GetTerminal<std::list<DataState::GlobalPosition*>>("SensorVertices");
}

void SensorVertices_Local::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    positionFrame = datagram.GetTerminal<Data::PositionalFrame>("PositionFrame");
    verticeLocations = datagram.GetTerminal<std::list<DataState::LocalPosition*>>("SensorVertices");
}


SensorVertices_Global::SensorVertices_Global()
{
    this->positionFrame = Data::PositionalFrame::GLOBAL;
    this->sensorName = "";
}
SensorVertices_Global::SensorVertices_Global(const std::string &sensorName)
{
    this->positionFrame = Data::PositionalFrame::GLOBAL;
    this->sensorName = sensorName;
}


SensorVertices_Local::SensorVertices_Local()
{
    this->positionFrame = Data::PositionalFrame::LOCAL;
    this->sensorName = "";
}
SensorVertices_Local::SensorVertices_Local(const std::string &sensorName)
{
    this->positionFrame = Data::PositionalFrame::LOCAL;
    this->sensorName = sensorName;
}

template <class T>
void SensorVerticesBase<T>::insertSensorVertice(T *verticePosition)
{
    verticeLocations.push_back(verticePosition);
}

template <class T>
std::list<T*> SensorVerticesBase<T>::getSensorVertices()
{
    return verticeLocations;
}

template <class T>
void SensorVerticesBase<T>::setSensorVertices(const std::list<T*> verticeList)
{
    verticeLocations = verticeList;
}

} //end of namespace DataVehicleSensors
template class DataVehicleSensors::SensorVerticesBase<DataState::GlobalPosition>;
template class DataVehicleSensors::SensorVerticesBase<DataState::LocalPosition>;



