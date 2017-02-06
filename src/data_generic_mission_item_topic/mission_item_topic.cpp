#include "mission_item_topic.h"


namespace MissionTopic {

const char MissionItemTopic_name[] = "missionItemTopic";
const MaceCore::TopicComponentStructure MissionItemTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MissionType>("missionType");
    structure.AddTerminal<MissionItem::AbstractMissionItem*>("missionItem");
    return structure;
}();

MaceCore::TopicDatagram MissionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MissionType>("missionType",missionType);
    datagram.AddTerminal<MissionItem::AbstractMissionItem*>("missionItem", missionItem);
    return datagram;
}

void MissionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionType = datagram.GetTerminal<MissionType>("missionType");
    missionItem = datagram.GetTerminal<MissionItem::AbstractMissionItem*>("missionItem");
}

MissionItemTopic::MissionItemTopic()
{

}

MissionItemTopic::MissionItemTopic(const MissionType &missionType)
{
    this->missionType = missionType;
}

void MissionItemTopic::setMissionItem(MissionItem::AbstractMissionItem *missionItem)
{
    this->missionItem = missionItem;
}

MissionItem::AbstractMissionItem* MissionItemTopic::getMissionItem()
{
    return missionItem;
}

}
