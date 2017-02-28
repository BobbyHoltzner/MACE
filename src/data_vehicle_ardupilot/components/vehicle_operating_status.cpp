#include "vehicle_operating_status.h"

namespace DataArdupilot
{

const char VehicleOperatingStatus_name[] = "vehicle_operating_status";
const MaceCore::TopicComponentStructure VehicleOperatingStatus_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<bool>("vehicleArmed");

    return structure;
}();


MaceCore::TopicDatagram VehicleOperatingStatus::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<bool>("vehicleArmed", vehicleArmed);

    return datagram;
}


void VehicleOperatingStatus::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    vehicleArmed = datagram.GetTerminal<bool>("vehicleArmed");
}


} //end of namespace
