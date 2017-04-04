#include "mission_item_topic.h"
namespace MissionTopic{

const char MissionItemTopic_name[] = "missionItem";
const MaceCore::TopicComponentStructure MissionItemTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<MissionType>("missionType");
    structure.AddTerminal<std::shared_ptr<MissionItem::AbstractMissionItem>>("missionItem");
    return structure;
}();

MaceCore::TopicDatagram MissionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<MissionType>("missionType",missionType);
    datagram.AddTerminal<std::shared_ptr<MissionItem::AbstractMissionItem>>("missionItem", missionItem);
    return datagram;
}

void MissionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    missionType = datagram.GetTerminal<MissionType>("missionType");
    missionItem = datagram.GetTerminal<std::shared_ptr<MissionItem::AbstractMissionItem>>("missionItem");
}


MissionItemTopic::MissionItemTopic()
{

}

MissionItemTopic::MissionItemTopic(const MissionType &missionType)
{
    this->missionType = missionType;
}

void MissionItemTopic::setMissionItem(const std::shared_ptr<MissionItem::AbstractMissionItem> missionItem)
{
    this->vehicleID = missionItem->getVehicleID();
    this->missionItem = missionItem;
}

std::shared_ptr<MissionItem::AbstractMissionItem> MissionItemTopic::getMissionItem()
{
    return missionItem;
}

} //end of namespace MissionTopic
