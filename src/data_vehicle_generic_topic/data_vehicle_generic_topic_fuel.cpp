#include "data_vehicle_generic_topic_fuel.h"

namespace DataVehicleGenericTopic {

const char GenericVehicleTopicFuel_name[] = "vehicleFuel";
const MaceCore::TopicComponentStructure GenericVehicleTopicFuel_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("voltage");
    structure.AddTerminal<double>("current");
    structure.AddTerminal<double>("remaining");
    return structure;
}();

MaceCore::TopicDatagram DataVehicleGenericTopic_Fuel::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("voltage", voltage);
    datagram.AddTerminal<double>("current", current);
    datagram.AddTerminal<double>("remaining", batteryRemaing);
    return datagram;
}

void DataVehicleGenericTopic_Fuel::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    voltage = datagram.GetTerminal<double>("voltage");
    current = datagram.GetTerminal<double>("current");
    batteryRemaing = datagram.GetTerminal<double>("remaining");
}

} //end of namespace DataVehicleGenericTopic
