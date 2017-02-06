#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include <list>
#include <string>

#include "data/i_topic_component_data_object.h"

#include "data_generic_mission_item/abstract_mission_item.h"

#include "command_type.h"

namespace MissionTopic
{

extern const char MissionItemTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionItemTopic_Structure;

class MissionItemTopic : public Data::NamedTopicComponentDataObject<MissionItemTopic_Name, &MissionItemTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    MissionItemTopic(const MissionType &missionType);

    void setMissionItem(const MissionItem::AbstractMissionItem* missionItem);

    MissionItem::AbstractMissionItem* getMissionItem();

private:
    MissionType missionType;
    MissionItem::AbstractMissionItem* missionItem;
};

} //end of namespace MissionTopic
#endif // MISSION_ITEM_TOPIC_H
