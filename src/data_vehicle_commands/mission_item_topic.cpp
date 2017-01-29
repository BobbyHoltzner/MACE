#include "mission_item_topic.h"

namespace DataVehicleCommands {

const char MissionItemTopic_Name[] = "MissionItemTopic";

const MaceCore::TopicComponentStructure MissionItemTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<GeneralMissionItem>("missionItem");
    return structure;
}();

MaceCore::TopicDatagram MissionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GeneralMissionItem>("missionItem", m_MissionItem);
    return datagram;
}

void MissionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_MissionItem = datagram.GetTerminal<GeneralMissionItem>("missionItem");
}

void MissionItemTopic::setMissionItem(const GeneralMissionItem &missionItem)
{
    m_MissionItem = missionItem;
}

} //end of namespace DataVehicleCommands

