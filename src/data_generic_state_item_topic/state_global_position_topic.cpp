#include "state_global_position_topic.h"

namespace DataStateTopic{

const char GlobalPositionTopic_name[] = "globalPosition";
const MaceCore::TopicComponentStructure GlobalPositionTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<double>("latitude");
    structure.AddTerminal<double>("longitude");
    structure.AddTerminal<double>("altitude");
    return structure;
}();

MaceCore::TopicDatagram StateGlobalPositionTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame", m_CoordinateFrame);
    datagram.AddTerminal<double>("latitude", latitude);
    datagram.AddTerminal<double>("longitude", longitude);
    datagram.AddTerminal<double>("altitude", altitude);
    return datagram;
}

void StateGlobalPositionTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    latitude = datagram.GetTerminal<double>("latitude");
    longitude = datagram.GetTerminal<double>("longitude");
    altitude = datagram.GetTerminal<double>("altitude");
}

StateGlobalPositionTopic::StateGlobalPositionTopic()
    :DataState::StateGlobalPosition()
{

}

StateGlobalPositionTopic::StateGlobalPositionTopic(const DataState::StateGlobalPosition &copyObj):
    DataState::StateGlobalPosition(copyObj)
{

}
} //end of namespace DataStateTopic
