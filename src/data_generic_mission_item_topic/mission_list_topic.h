#ifndef MISSION_LIST_TOPIC_H
#define MISSION_LIST_TOPIC_H

#include <list>
#include <string>

#include "data/i_topic_component_data_object.h"

#include "data_generic_mission_item/mission_list.h"

#include "command_type.h"

namespace MissionTopic
{

extern const char MissionListTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionListTopic_Structure;

class MissionListTopic : public Data::NamedTopicComponentDataObject<MissionListTopic_Name, &MissionListTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionListTopic(const MissionType &missionType);

private:
    MissionType missionType;
    MissionItem::MissionList* missionList;
};

} //end of namespace MissionTopic

#endif // MISSION_LIST_TOPIC_H
