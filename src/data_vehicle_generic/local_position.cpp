#include "local_position.h"

#include <math.h>

namespace DataVehicleGeneric
{


const char LocalPosition_name[] = "local_position";
const MaceCore::TopicComponentStructure LocalPosition_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    structure.AddTerminal<CoordinateFrame>("frame");

    return structure;
}();



MaceCore::TopicDatagram LocalPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<CoordinateFrame>("frame", frame);

    return datagram;
}


void LocalPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
    frame = datagram.GetTerminal<CoordinateFrame>("frame");
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
