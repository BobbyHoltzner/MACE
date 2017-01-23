#include "vehicle_operating_status.h"

namespace DataVehicleArdupilot
{

const char VehicleOperatingStatus_name[] = "vehicle_operating_status";
const MaceCore::TopicComponentStructure VehicleOperatingStatus_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<bool>("vehicleArmed");

    return structure;
}();


MaceCore::TopicDatagram VehicleOperatingStatus::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<bool>("vehicleArmed", m_Armed);

    return datagram;
}


void VehicleOperatingStatus::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_Armed = datagram.GetTerminal<bool>("vehicleArmed");
}


} //end of namespace
