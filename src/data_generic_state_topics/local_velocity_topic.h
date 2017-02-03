#ifndef LOCAL_VELOCITY_TOPIC_H
#define LOCAL_VELOCITY_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_items/local_velocity.h"

namespace DataStateTopic
{

extern const char LocalVelocity_name[];
extern const MaceCore::TopicComponentStructure LocalVelocity_structure;

class LocalVelocityTopic : public DataState::LocalVelocity, public Data::NamedTopicComponentDataObject<LocalVelocity_name, &LocalVelocity_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

}

#endif // DATAVEHICLEGENERIC_LOCALVELOCITY_H
