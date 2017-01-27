#include "vehicle_operating_parameters.h"

namespace DataVehicleArdupilot
{

const char VehicleOperatingParameters_name[] = "vehicle_operating_parameters";
const MaceCore::TopicComponentStructure VehicleOperatingParameters_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleType");
    structure.AddTerminal<uint32_t>("flightMode");

    return structure;
}();


MaceCore::TopicDatagram VehicleOperatingParameters::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleType", (int)m_Platform);
    datagram.AddTerminal<uint32_t>("flightMode", m_FlightMode);

    return datagram;
}


void VehicleOperatingParameters::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_Platform = (Arduplatforms)datagram.GetTerminal<int>("vehicleType");
    m_FlightMode = datagram.GetTerminal<uint32_t>("flightMode");
}

void VehicleOperatingParameters::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    this->setPlatform((Arduplatforms)msg.type);
    this->setFlightMode(msg.custom_mode);
}


}
