#include "mission_home_topic.h"

namespace MissionTopic{

const char MissionHomeTopic_name[] = "VehicleHome";
const MaceCore::TopicComponentStructure MissionHomeTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("vehicleID");
    structure.AddTerminal<MissionItem::SpatialHome>("homeItem");
    return structure;
}();

MaceCore::TopicDatagram MissionHomeTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("vehicleID",vehicleID);
    datagram.AddTerminal<MissionItem::SpatialHome>("homeItem", vehicleHome);
    return datagram;
}

void MissionHomeTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<int>("vehicleID");
    vehicleHome = datagram.GetTerminal<MissionItem::SpatialHome>("homeItem");
}

MissionHomeTopic::MissionHomeTopic()
{

}

} //end of namespace MissionTopic
