#include "local_velocity.h"

#include <math.h>

namespace DataVehicleGeneric
{


const char LocalVelocity_name[] = "local_velocity";
const MaceCore::TopicComponentStructure LocalVelocity_structure = [](){
        MaceCore::TopicComponentStructure structure;
        structure.AddTerminal<double>("x");
        structure.AddTerminal<double>("y");
        structure.AddTerminal<double>("z");
        structure.AddTerminal<Data::CoordinateFrame>("frame");

        return structure;
    }();




MaceCore::TopicDatagram LocalVelocity::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    datagram.AddTerminal<Data::CoordinateFrame>("frame", frame);

    return datagram;
}


void LocalVelocity::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
    frame = datagram.GetTerminal<Data::CoordinateFrame>("frame");
}

}
