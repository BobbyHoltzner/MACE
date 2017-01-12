#include "global_velocity.h"

#include <math.h>

namespace DataVehicleGeneric
{

const char GlobalVelocity_name[] = "global_velocity";
const MaceCore::TopicComponentStructure GlobalVelocity_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    structure.AddTerminal<double>("heading");
    structure.AddTerminal<CoordinateFrame>("frame");

    return structure;
}();




MaceCore::TopicDatagram GlobalVelocity::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<double>("heading", heading);
    datagram.AddTerminal<CoordinateFrame>("frame", frame);

    return datagram;
}


void GlobalVelocity::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
    heading = datagram.GetTerminal<double>("heading");
    frame = datagram.GetTerminal<CoordinateFrame>("frame");
}

}
