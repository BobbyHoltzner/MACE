#include "general_mission_item.h"

namespace DataVehicleCommands {

const char CommandVehicleItem_Name[] = "VehicleCommandItem";
const MaceCore::TopicComponentStructure CommandVehicleItem_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<GeneralMissionItem>("missionItem");
    return structure;
}();

MaceCore::TopicDatagram GeneralMissionItem::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("mode", m_CommandVehicleMode);
    return datagram;
}

void GeneralMissionItem::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

}

} //end of namespace DataVehicleCommands
