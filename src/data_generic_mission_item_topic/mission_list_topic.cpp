#include "mission_list_topic.h"

namespace MissionTopic {

const char MissionListTopic_name[] = "missionListTopic";
const MaceCore::TopicComponentStructure MissionListTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MissionType>("missionType");
    structure.AddTerminal<std::list<MissionItem::AbstractMissionItem*>>("missionList");

    return structure;
}();

MaceCore::TopicDatagram MissionListTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MissionType>("missionType",missionType);
    datagram.AddTerminal<std::list<MissionItem::AbstractMissionItem*>>("missionList", missionList);
    return datagram;
}

void MissionListTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionType = datagram.GetTerminal<MissionType>("missionType");
    missionList = datagram.GetTerminal<std::list<MissionItem::AbstractMissionItem*>>("missionList");
}

MissionListTopic::MissionListTopic(const MissionType &missionType)
{
    this->missionType = missionType;
}

}
