#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "general_mission_item.h"

namespace DataVehicleCommands
{

extern const char MissionItemTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionItemTopic_Structure;

class MissionItemTopic : public Data::NamedTopicComponentDataObject<MissionItemTopic_Name, &MissionItemTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    void setMissionItem(const GeneralMissionItem &missionItem);

private:
    GeneralMissionItem m_MissionItem;
};
} //end of namespace DataVehicleCommands
#endif // MISSION_ITEM_TOPIC_H
