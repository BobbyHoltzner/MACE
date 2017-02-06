#ifndef MISSION_LIST_TOPIC_H
#define MISSION_LIST_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "command_mission_type.h"
#include "data_generic_mission_item/mission_list.h"

namespace MissionTopic{

extern const char MissionListTopic_name[];
extern const MaceCore::TopicComponentStructure MissionListTopic_structure;

class MissionListTopic :public Data::NamedTopicComponentDataObject<MissionListTopic_name, &MissionListTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    MissionListTopic();
    MissionListTopic(const MissionType &missionType);

private:
    MissionType missionType;
    MissionItem::MissionList* missionList;
};

} //end of namespace MissionTopic

#endif // MISSION_LIST_TOPIC_H
