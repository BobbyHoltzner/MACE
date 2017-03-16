#include "state_local_position_topic.h"

namespace DataStateTopic{

const char LocalPositionTopic_name[] = "localPosition";
const MaceCore::TopicComponentStructure LocalPositionTopic_structure = [](){
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::PositionalFrame>("PositionFrame");
    structure.AddTerminal<Data::CoordinateFrame>("CoordinateFrame");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    return structure;
}();



MaceCore::TopicDatagram StateLocalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::PositionalFrame>("PositionFrame", m_PositionFrame);
    datagram.AddTerminal<Data::CoordinateFrame>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);
    return datagram;
}


void StateLocalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_PositionFrame = datagram.GetTerminal<Data::PositionalFrame>("PositionFrame");
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrame>("CoordinateFrame");
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

StateLocalPositionTopic::StateLocalPositionTopic()
    :DataState::StateLocalPosition()
{

}

StateLocalPositionTopic::StateLocalPositionTopic(const DataState::StateLocalPosition &copyObj):
    DataState::StateLocalPosition(copyObj)
{

}

} //end of namespace DataStateTopic
