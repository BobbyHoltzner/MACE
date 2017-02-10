#include "data_vehicle_generic_topic_text.h"

namespace DataVehicleGenericTopic {

const char GenericVehicleTopicText_name[] = "statusText";
const MaceCore::TopicComponentStructure GenericVehicleTopicText_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<DataVehicleGenericTopic_Text::STATUS_SEVERITY>("fix");
    structure.AddTerminal<std::string>("text");
    return structure;
}();

MaceCore::TopicDatagram DataVehicleGenericTopic_Text::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<STATUS_SEVERITY>("severity", severity);
    datagram.AddTerminal<std::string>("text", dataString);
    return datagram;
}

void DataVehicleGenericTopic_Text::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    severity = datagram.GetTerminal<STATUS_SEVERITY>("severity");
    dataString = datagram.GetTerminal<std::string>("text");
}

} //end of namespace DataVehicleGenericTopic
