#include "mission_list_topic.h"

namespace MissionTopic{

const char MissionListTopic_name[] = "missionList";
const MaceCore::TopicComponentStructure MissionListTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<MissionType>("missionType");
    structure.AddTerminal<std::shared_ptr<MissionItem::MissionList>>("missionList");
    return structure;
}();

MaceCore::TopicDatagram MissionListTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<MissionType>("missionType",missionType);
    datagram.AddTerminal<std::shared_ptr<MissionItem::MissionList>>("missionList", missionList);
    return datagram;
}

void MissionListTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    missionType = datagram.GetTerminal<MissionType>("missionType");
    missionList = datagram.GetTerminal<std::shared_ptr<MissionItem::MissionList>>("missionList");
}

MissionListTopic::MissionListTopic()
{

}

MissionListTopic::MissionListTopic(const MissionType &missionType)
{
    this->missionType = missionType;
}


void MissionListTopic::setMissionList(const std::shared_ptr<MissionItem::MissionList> missionList)
{
    this->missionList = missionList;
}

std::shared_ptr<MissionItem::MissionList> MissionListTopic::getMissionList()
{
    return missionList;
}

} //end of namespace MissionTopic
