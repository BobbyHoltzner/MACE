#include "mission_item_request_topic.h"

namespace MissionTopic{

const char MissionItemRequestTopic_name[] = "MissionItemRequest";
const MaceCore::TopicComponentStructure MissionItemRequestTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<int>("missionIndex");
    return structure;
}();

MaceCore::TopicDatagram MissionItemRequestTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<MissionType>("missionIndex",missionItemIndex);
    return datagram;
}

void MissionItemRequestTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    missionItemIndex = datagram.GetTerminal<MissionType>("missionIndex");
}

void MissionItemRequestTopic::MissionItemRequestTopic(const int &vehicleID, const int &missionItemIndex)
{
    this->vehicleID = vehicleID;
    this->missionItemIndex = missionItemIndex;
}

} //end of namespace MissionTopic
