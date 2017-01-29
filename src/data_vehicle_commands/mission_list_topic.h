#ifndef MISSION_LIST_TOPIC_H
#define MISSION_LIST_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "missionCommandComponents/command_mission_list.h"

namespace DataVehicleCommands
{

extern const char MissionListTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionListTopic_Structure;

class MissionListTopic : public Data::NamedTopicComponentDataObject<MissionListTopic_Name, &MissionListTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:


private:
    CommandMissionList m_CommandMissionList;
};
} //end of namespace DataVehicleCommands
#endif // MISSION_LIST_TOPIC_H
