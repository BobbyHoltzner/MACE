#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "command_mission_type.h"
#include "data_generic_mission_item/abstract_mission_item.h"

namespace MissionTopic{

extern const char MissionItemTopic_name[];
extern const MaceCore::TopicComponentStructure MissionItemTopic_structure;

class MissionItemTopic :public Data::NamedTopicComponentDataObject<MissionItemTopic_name, &MissionItemTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemTopic();
    MissionItemTopic(const MissionType &missionType);

    void setMissionItem(MissionItem::AbstractMissionItem *missionItem);

    MissionItem::AbstractMissionItem* getMissionItem();

private:
    MissionType missionType;
    MissionItem::AbstractMissionItem* missionItem;
};

} //end of namespace MissionTopic
#endif // MISSION_ITEM_TOPIC_H
