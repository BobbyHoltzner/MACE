#include "data_generic_item_topic_flightmode.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicFlightMode_name[] = "vehicle_operating_parameters";
const MaceCore::TopicComponentStructure DataGenericItemTopicFlightMode_structure = []{
    MaceCore::TopicComponentStructure structure;
    //structure.AddTerminal<Data::VehicleTypes>("vehicleType");
    structure.AddTerminal<std::string>("flightMode");
    structure.AddTerminal<bool>("vehicleArmed");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_FlightMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    //datagram.AddTerminal<Data::VehicleTypes>("vehicleType", vehicleType);
    datagram.AddTerminal<std::string>("flightMode", flightMode);
    datagram.AddTerminal<bool>("vehicleArmed", vehicleArmed);
    return datagram;
}


void DataGenericItemTopic_FlightMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    //vehicleType = datagram.GetTerminal<Data::VehicleTypes>("vehicleType");
    flightMode = datagram.GetTerminal<std::string>("flightMode");
    vehicleArmed = datagram.GetTerminal<bool>("vehicleArmed");

}

} //end of namespace DataGenericItemTopic
