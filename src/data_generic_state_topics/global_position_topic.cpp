#include "global_position_topic.h"

#include <math.h>

namespace DataStateTopic
{
const char GlobalPosition_name[] = "global_position";
const MaceCore::TopicComponentStructure GlobalPosition_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrame>("CoordinateFrame");
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<double>("altitude");

    return structure;
}();




MaceCore::TopicDatagram GlobalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrame>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("latitude", latitude);
    datagram.AddTerminal<double>("longitude", longitude);
    datagram.AddTerminal<double>("altitude", altitude);

    return datagram;
}


void GlobalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrame>("CoordinateFrame");
    latitude = datagram.GetTerminal<double>("latitude");
    longitude = datagram.GetTerminal<double>("longitude");
    altitude = datagram.GetTerminal<double>("altitude");
}

}
