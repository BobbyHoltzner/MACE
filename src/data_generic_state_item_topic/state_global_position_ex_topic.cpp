#include "state_global_position_ex_topic.h"

namespace DataStateTopic{

const char GlobalPositionTopicEx_name[] = "globalPositionEx";
const MaceCore::TopicComponentStructure GlobalPositionTopicEx_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::PositionalFrame>("PositionFrame");
    structure.AddTerminal<Data::CoordinateFrame>("CoordinateFrame");
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<double>("altitude");
    structure.AddTerminal<double>("heading");
    return structure;
}();

MaceCore::TopicDatagram StateGlobalPositionExTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::PositionalFrame>("PositionFrame", m_PositionFrame);
    datagram.AddTerminal<Data::CoordinateFrame>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("latitude", latitude);
    datagram.AddTerminal<double>("longitude", longitude);
    datagram.AddTerminal<double>("altitude", altitude);
    datagram.AddTerminal<double>("heading", heading);
    return datagram;
}

void StateGlobalPositionExTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_PositionFrame = datagram.GetTerminal<Data::PositionalFrame>("PositionFrame");
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrame>("CoordinateFrame");
    latitude = datagram.GetTerminal<double>("latitude");
    longitude = datagram.GetTerminal<double>("longitude");
    altitude = datagram.GetTerminal<double>("altitude");
    heading = datagram.GetTerminal<double>("heading");
}

StateGlobalPositionExTopic::StateGlobalPositionExTopic()
    :DataState::StateGlobalPositionEx()
{

}

StateGlobalPositionExTopic::StateGlobalPositionExTopic(const DataState::StateGlobalPositionEx &copyObj):
    DataState::StateGlobalPositionEx(copyObj)
{

}
} //end of namespace DataStateTopic
