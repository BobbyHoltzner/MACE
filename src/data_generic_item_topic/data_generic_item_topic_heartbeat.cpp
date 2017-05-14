#include "data_generic_item_topic_heartbeat.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicHeartbeat_name[] = "heartbeat";
const MaceCore::TopicComponentStructure DataGenericItemTopicHeartbeat_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<CommsProtocol>("protocol");
    structure.AddTerminal<SystemType>("type");
    structure.AddTerminal<AutopilotType>("autopilot");
    structure.AddTerminal<bool>("maceCompanion");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Heartbeat::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<CommsProtocol>("protocol", protocol);
    datagram.AddTerminal<SystemType>("type", type);
    datagram.AddTerminal<AutopilotType>("autopilot", autopilot);
    datagram.AddTerminal<bool>("maceCompanion", maceCompanion);
    return datagram;
}

void DataGenericItemTopic_Heartbeat::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    protocol = datagram.GetTerminal<CommsProtocol>("protocol");
    type = datagram.GetTerminal<SystemType>("type");
    autopilot = datagram.GetTerminal<AutopilotType>("autopilot");
    maceCompanion = datagram.GetTerminal<bool>("maceCompanion");
}

DataGenericItemTopic_Heartbeat::DataGenericItemTopic_Heartbeat()
    :DataGenericItem::DataGenericItem_Heartbeat()
{

}

DataGenericItemTopic_Heartbeat::DataGenericItemTopic_Heartbeat(const DataGenericItem::DataGenericItem_Heartbeat &copyObj):
    DataGenericItem::DataGenericItem_Heartbeat(copyObj)
{

}

} //end of namespace DataGenericItemTopic
