#include "data_generic_item_topic_fuel.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicFuel_name[] = "vehicleFuel";
const MaceCore::TopicComponentStructure DataGenericItemTopicFuel_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("voltage");
    structure.AddTerminal<double>("current");
    structure.AddTerminal<double>("remaining");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Fuel::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("voltage", voltage);
    datagram.AddTerminal<double>("current", current);
    datagram.AddTerminal<double>("remaining", batteryRemaing);
    return datagram;
}

void DataGenericItemTopic_Fuel::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    voltage = datagram.GetTerminal<double>("voltage");
    current = datagram.GetTerminal<double>("current");
    batteryRemaing = datagram.GetTerminal<double>("remaining");
}

} //end of namespace DataGenericItemTopic
