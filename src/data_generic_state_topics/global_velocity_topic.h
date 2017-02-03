#ifndef GLOBAL_VELOCITY_TOPIC_H
#define GLOBAL_VELOCITY_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_items/global_velocity.h"

namespace DataStateTopic
{

extern const char GlobalVelocity_name[];
extern const MaceCore::TopicComponentStructure GlobalVelocity_structure;

class GlobalVelocityTopic : public DataState::GlobalVelocity, public Data::NamedTopicComponentDataObject<GlobalVelocity_name, &GlobalVelocity_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

}

#endif // GLOBAL_VELOCITY_TOPIC_H
