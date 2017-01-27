#include "vehicle_flightMode.h"

namespace DataVehicleArdupilot
{

const char VehicleOperatingParameters_name[] = "vehicle_operating_parameters";
const MaceCore::TopicComponentStructure VehicleOperatingParameters_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleType");
    structure.AddTerminal<uint32_t>("flightMode");

    return structure;
}();


MaceCore::TopicDatagram VehicleFlightMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleType", (int)m_VehicleType);
    datagram.AddTerminal<uint32_t>("flightMode", m_FlightMode);

    return datagram;
}


void VehicleFlightMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_VehicleType = (VehicleTypes)datagram.GetTerminal<int>("vehicleType");
    m_FlightMode = datagram.GetTerminal<uint32_t>("flightMode");
}

void VehicleFlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    this->setVehicleType(msg.type);
    this->setFlightMode(msg.custom_mode);
}

void VehicleFlightMode::setVehicleType(int vehicleType){
        switch (vehicleType) {
        case MAV_TYPE_FIXED_WING:
        {
            m_VehicleType = VehicleTypes::PLANE;
            break;
        }
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        {
            m_VehicleType = VehicleTypes::COPTER;
            break;
        }
        default:
            break;
        }
}

} //end of namespace DataVehicleArdupilot
