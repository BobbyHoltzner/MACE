#include "command_vehicle_mode.h"

namespace DataVehicleCommands {

const char CommandVehicleMode_Name[] = "CommandVehicleMode";
const MaceCore::TopicComponentStructure CommandVehicleMode_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("mode");
    return structure;
}();

MaceCore::TopicDatagram CommandVehicleMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("mode", m_CommandVehicleMode);
    return datagram;
}

void CommandVehicleMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CommandVehicleMode = datagram.GetTerminal<std::string>("mode");
}

}





