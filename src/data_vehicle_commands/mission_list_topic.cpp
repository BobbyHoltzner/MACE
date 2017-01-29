#include "mission_list_topic.h"

namespace DataVehicleCommands {

const char MissionListTopic_Name[] = "MissionListTopic";

const MaceCore::TopicComponentStructure MissionListTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<CommandMissionList>("missionList");
    return structure;
}();

MaceCore::TopicDatagram MissionListTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<CommandMissionList>("missionList", m_CommandMissionList);
    return datagram;
}

void MissionListTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CommandMissionList = datagram.GetTerminal<CommandMissionList>("missionList");
}

} //end of namespace DataVehicleCommands

