#include "local_position.h"

#include <math.h>

namespace DataVehicleGeneric
{


const char LocalPosition_name[] = "local_position";
const MaceCore::TopicComponentStructure LocalPosition_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<PositionFrame>("PositionFrame");
    structure.AddTerminal<CoordinateFrame>("CoordinateFrame");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    return structure;
}();



MaceCore::TopicDatagram LocalPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<PositionFrame>("PositionFrame", m_PositionFrame);
    datagram.AddTerminal<CoordinateFrame>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    return datagram;
}


void LocalPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_PositionFrame = datagram.GetTerminal<PositionFrame>("PositionFrame");
    m_CoordinateFrame = datagram.GetTerminal<CoordinateFrame>("CoordinateFrame");
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

LocalPosition::LocalPosition():
    Position(PositionFrame::LOCAL)
{

}

LocalPosition::LocalPosition(const CoordinateFrame &frame):
    Position(PositionFrame::LOCAL, frame)
{

}

LocalPosition::LocalPosition(const double &x, const double &y, const double &z):
    Position(PositionFrame::LOCAL)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

LocalPosition::LocalPosition(const CoordinateFrame &frame, const double &x, const double &y, const double &z):
    Position(PositionFrame::LOCAL,frame)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

double LocalPosition::bearingBetween(const LocalPosition &position)
{
    throw std::runtime_error("Not Implimented");
    return 0.0;
}

double LocalPosition::initialBearing(const LocalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

double LocalPosition::finalBearing(const LocalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

double LocalPosition::distanceBetween(const LocalPosition &position)
{
    double deltaX = position.x - this->x;
    double deltaY = position.y - this->y;
    double deltaZ = position.z - this->z;

    return sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}

}
