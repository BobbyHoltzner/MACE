#ifndef GLOBAL_POSITION_TOPIC_H
#define GLOBAL_POSITION_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_items/global_position.h"

namespace DataStateTopic
{

extern const char GlobalPosition_name[];
extern const MaceCore::TopicComponentStructure GlobalPosition_structure;

class GlobalPositionTopic : public DataState::GlobalPosition, public Data::NamedTopicComponentDataObject<GlobalPosition_name, &GlobalPosition_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

}


#endif // GLOBAL_POSITION_TOPIC_H
