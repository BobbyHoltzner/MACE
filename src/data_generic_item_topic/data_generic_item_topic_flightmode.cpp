#include "data_generic_item_topic_flightmode.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicFlightMode_name[] = "vehicle_operating_parameters";
const MaceCore::TopicComponentStructure DataGenericItemTopicFlightMode_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::VehicleTypes>("vehicleType");
    structure.AddTerminal<Data::AutopilotTypes>("autopilotType");
    structure.AddTerminal<std::string>("flightModeString");
    structure.AddTerminal<int>("flightModeInt");
    structure.AddTerminal<bool>("vehicleArmed");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_FlightMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::VehicleTypes>("vehicleType", vehicleType);
    datagram.AddTerminal<Data::AutopilotTypes>("autopilotType", autopilotType);
    datagram.AddTerminal<std::string>("flightModeString", flightModeString);
    datagram.AddTerminal<int>("flightModeInt", flightModeInt);
    datagram.AddTerminal<bool>("vehicleArmed", vehicleArmed);
    return datagram;
}


void DataGenericItemTopic_FlightMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    vehicleType = datagram.GetTerminal<Data::VehicleTypes>("vehicleType");
    autopilotType = datagram.GetTerminal<Data::AutopilotTypes>("autopilotType");
    flightModeString = datagram.GetTerminal<std::string>("flightModeString");
    flightModeInt = datagram.GetTerminal<int>("flightModeInt");
    vehicleArmed = datagram.GetTerminal<bool>("vehicleArmed");
}

DataGenericItemTopic_FlightMode::DataGenericItemTopic_FlightMode()
    :DataGenericItem::DataGenericItem_FlightMode()
{

}

DataGenericItemTopic_FlightMode::DataGenericItemTopic_FlightMode(const DataGenericItem::DataGenericItem_FlightMode &copyObj):
    DataGenericItem::DataGenericItem_FlightMode(copyObj)
{

}

} //end of namespace DataGenericItemTopic
