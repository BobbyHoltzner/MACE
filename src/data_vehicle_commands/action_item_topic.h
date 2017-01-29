#ifndef ACTION_ITEM_TOPIC_H
#define ACTION_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "general_action_item.h"

namespace DataVehicleCommands
{

extern const char ActionItemTopic_Name[];
extern const MaceCore::TopicComponentStructure ActionItemTopic_Structure;

class ActionItemTopic : public Data::NamedTopicComponentDataObject<ActionItemTopic_Name, &ActionItemTopic_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    void setActionItem(const GeneralActionItem &actionItem);

private:
    GeneralActionItem m_ActionItem;
};
} //end of namespace DataVehicleCommands
#endif // ACTION_ITEM_TOPIC_H
