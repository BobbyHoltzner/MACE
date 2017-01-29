#ifndef MISSION_ITEM_TOPIC_H
#define MISSION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "abstract_mission_command.h"

namespace DataVehicleCommands
{

extern const char MissionCommandTopic_Name[];
extern const MaceCore::TopicComponentStructure MissionCommandTopic_Structure;

class MissionCommandTopic : public Data::NamedTopicComponentDataObject<MissionCommandTopic_Name, &MissionCommandTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    void setActionItem(const std::shared_ptr<AbstractMissionCommand> &actionItem);

    std::shared_ptr<AbstractMissionCommand> getActionItem() const;

    MissionItemTypes getActionItemType() const;

private:
    MissionItemTypes m_MissionCommandType;
    AbstractMissionCommand m_MissionItem;
};
} //end of namespace DataVehicleCommands
#endif // MISSION_ITEM_TOPIC_H
