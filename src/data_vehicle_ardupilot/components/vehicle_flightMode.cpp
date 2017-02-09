#include "vehicle_flightMode.h"

namespace DataVehicleArdupilot
{

const char VehicleOperatingParameters_name[] = "vehicle_operating_parameters";
const MaceCore::TopicComponentStructure VehicleOperatingParameters_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleType");
    structure.AddTerminal<int>("flightMode");
    return structure;
}();


MaceCore::TopicDatagram VehicleFlightMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleType", (int)m_VehicleType);
    datagram.AddTerminal<int>("flightMode", m_FlightMode);
    return datagram;
}


void VehicleFlightMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_VehicleType = (Data::VehicleTypes)datagram.GetTerminal<int>("vehicleType");
    m_FlightMode = datagram.GetTerminal<uint32_t>("flightMode");
}

int VehicleFlightMode::getFlightMode(const std::string &flightMode){

    std::map<int,std::string> availableFM;

    if(m_VehicleType == Data::VehicleTypes::PLANE){
        availableFM = arduplaneFM;
    }else{
        availableFM = arducopterFM;
    }

    std::map<int,std::string>::iterator it;
    int vehicleModeID = 0;

    for (it=availableFM.begin(); it != availableFM.end(); it++)
    {
        if(it->second == flightMode)
        {
            vehicleModeID = it->first;
            return vehicleModeID;
        }
    }
    //Probably dont want to error here but what would be the safest method
    throw std::runtime_error("The flight mode provided does not exist");
}

void VehicleFlightMode::getAvailableFlightModes(const Data::VehicleTypes &vehicleType, std::map<int, std::string> &availableFM)
{

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
            m_VehicleType = Data::VehicleTypes::PLANE;
            break;
        }
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        {
            m_VehicleType = Data::VehicleTypes::COPTER;
            break;
        }
        default:
            break;
        }
}

} //end of namespace DataVehicleArdupilot
