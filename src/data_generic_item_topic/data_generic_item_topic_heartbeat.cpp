#include "data_generic_item_topic_heartbeat.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicHeartbeat_name[] = "heartbeat";
const MaceCore::TopicComponentStructure DataGenericItemTopicHeartbeat_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MAV_PROTOCOL>("protocol");
    structure.AddTerminal<MAV_TYPE>("type");
    structure.AddTerminal<MAV_AUTOPILOT>("autopilot");
    structure.AddTerminal<bool>("maceCompanion");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Heartbeat::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MAV_PROTOCOL>("protocol", protocol);
    datagram.AddTerminal<MAV_TYPE>("type", type);
    datagram.AddTerminal<MAV_AUTOPILOT>("autopilot", autopilot);
    datagram.AddTerminal<bool>("maceCompanion", maceCompanion);
    return datagram;
}

void DataGenericItemTopic_Heartbeat::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    protocol = datagram.GetTerminal<MAV_PROTOCOL>("protocol");
    type = datagram.GetTerminal<MAV_TYPE>("type");
    autopilot = datagram.GetTerminal<MAV_AUTOPILOT>("autopilot");
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
