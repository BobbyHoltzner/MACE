#include "sensor_vertices.h"

namespace DataVehicleSensors
{
const char SensorVertices_Name[] = "SensorVertices";

const MaceCore::TopicComponentStructure SensorVertices_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<DataVehicleGeneric::PositionFrame>("PositionFrame");
    structure.AddTerminal<std::list<DataVehicleGeneric::Position*>>("SensorVertices");
    return structure;
}();

template <>
MaceCore::TopicDatagram SensorVertices<DataVehicleGeneric::GlobalPosition>::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<DataVehicleGeneric::PositionFrame>("PositionFrame",positionFrame);
    datagram.AddTerminal<std::list<DataVehicleGeneric::GlobalPosition*>>("SensorVertices",verticeLocations);
    return datagram;
}

template <>
MaceCore::TopicDatagram SensorVertices<DataVehicleGeneric::LocalPosition>::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<DataVehicleGeneric::PositionFrame>("PositionFrame",positionFrame);
    datagram.AddTerminal<std::list<DataVehicleGeneric::LocalPosition*>>("SensorVertices",verticeLocations);
    return datagram;
}

template <>
void SensorVertices<DataVehicleGeneric::GlobalPosition>::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<double>("SensorName");
    positionFrame = datagram.GetTerminal<DataVehicleGeneric::PositionFrame>("PositionFrame");
    verticeLocations = datagram.GetTerminal<std::list<DataVehicleGeneric::GlobalPosition*>>("SensorVertices");
}

template <>
void SensorVertices<DataVehicleGeneric::LocalPosition>::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<double>("SensorName");
    positionFrame = datagram.GetTerminal<DataVehicleGeneric::PositionFrame>("PositionFrame");
    verticeLocations = datagram.GetTerminal<std::list<DataVehicleGeneric::LocalPosition*>>("SensorVertices");
}


template<>
SensorVertices<DataVehicleGeneric::GlobalPosition>::SensorVertices(const std::string &sensorName)
{
    this->positionFrame = DataVehicleGeneric::PositionFrame::GLOBAL;
    this->sensorName = sensorName;
}
template<>
SensorVertices<DataVehicleGeneric::LocalPosition>::SensorVertices(const std::string &sensorName)
{
    this->positionFrame = DataVehicleGeneric::PositionFrame::LOCAL;
    this->sensorName = sensorName;
}

template<class T>
std::list<T*> SensorVertices<T>::getSensorVertices()
{
    return verticeLocations;
}

template<class T>
void SensorVertices<T>::setSensorVertices(const std::list<T*> &sensorVertices){
    verticeLocations = sensorVertices;
}

} //end of namespace DataVehicleSensors
