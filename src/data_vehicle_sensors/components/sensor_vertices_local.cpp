#include "sensor_vertices_local.h"

namespace DataVehicleSensors
{
const char SensorVerticesLocal_Name[] = "SensorVerticesLocal";

const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<Data::PositionalFrame>("PositionFrame");
    structure.AddTerminal<std::vector<DataState::StateLocalPosition>>("SensorVertices");
    return structure;
}();

MaceCore::TopicDatagram SensorVertices_Local::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<Data::PositionalFrame>("PositionFrame",positionFrame);
    datagram.AddTerminal<std::vector<DataState::StateLocalPosition>>("SensorVertices",verticeLocations);
    return datagram;
}

void SensorVertices_Local::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    positionFrame = datagram.GetTerminal<Data::PositionalFrame>("PositionFrame");
    verticeLocations = datagram.GetTerminal<std::vector<DataState::StateLocalPosition>>("SensorVertices");
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


std::vector<DataState::StateLocalPosition> SensorVertices_Local::getSensorVertices() const
{
    return(verticeLocations);
}

void SensorVertices_Local::setSensorVertices(const std::vector<DataState::StateLocalPosition> &verticeVector)
{
    verticeLocations = verticeVector;
}

} //end of namespace DataVehicleSensors
