#include "mission_item_reached_topic.h"

namespace MissionTopic{

const char MissionItemReachedTopic_name[] = "MissionItemReached";
const MaceCore::TopicComponentStructure MissionItemReachedTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<int>("missionItemIndex");
    return structure;
}();

MaceCore::TopicDatagram MissionItemRequestTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<int>("missionItemIndex",missionItemIndex);
    return datagram;
}

void MissionItemReachedTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    missionItemIndex = datagram.GetTerminal<int>("missionItemIndex");
}

} //end of namespace MissionTopic
