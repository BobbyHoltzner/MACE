#include "mission_list_topic.h"

namespace MissionTopic{

const char MissionListTopic_name[] = "AT";
const MaceCore::TopicComponentStructure MissionListTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MissionType>("missionType");
    structure.AddTerminal<MissionItem::MissionList*>("missionList");
    return structure;
}();

MaceCore::TopicDatagram MissionListTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MissionType>("missionType",missionType);
    datagram.AddTerminal<MissionItem::MissionList*>("missionList", missionList);
    return datagram;
}

void MissionListTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionType = datagram.GetTerminal<MissionType>("missionType");
    missionList = datagram.GetTerminal<MissionItem::MissionList*>("missionList");
}

MissionListTopic::MissionListTopic()
{

}

MissionListTopic::MissionListTopic(const MissionType &missionType)
{
    this->missionType = missionType;
}

} //end of namespace MissionTopic
