#include "local_position_topic.h"

#include <math.h>

namespace DataStateTopic
{

const char LocalPosition_name[] = "local_position";
const MaceCore::TopicComponentStructure LocalPosition_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrame>("CoordinateFrame");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    return structure;
}();



MaceCore::TopicDatagram LocalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrame>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    return datagram;
}


void LocalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrame>("CoordinateFrame");
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

}
