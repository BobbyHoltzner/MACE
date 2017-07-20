#include "mission_item_current_topic.h"

namespace MissionTopic{

const char MissionItemCurrentTopic_name[] = "MissionItemCurrent";
const MaceCore::TopicComponentStructure MissionItemCurrentTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<int>("missionItemIndex");
    return structure;
}();

MaceCore::TopicDatagram MissionItemCurrentTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<int>("missionItemIndex",missionItemIndex);
    return datagram;
}

void MissionItemCurrentTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    missionItemIndex = datagram.GetTerminal<int>("missionItemIndex");
}

MissionItemCurrentTopic::MissionItemCurrentTopic():
    vehicleID(0), missionItemIndex(0)
{

}

MissionItemCurrentTopic::MissionItemCurrentTopic(const MissionItemCurrentTopic &copyObj)
{
    this->vehicleID = copyObj.vehicleID;
    this->missionItemIndex = copyObj.missionItemIndex;
}

} //end of namespace MissionTopic
