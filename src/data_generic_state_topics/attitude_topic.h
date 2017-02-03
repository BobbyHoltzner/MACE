#ifndef ATTITUDE_TOPIC_H
#define ATTITUDE_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_items/attitude.h"

namespace DataStateTopic
{

extern const char Attitude_name[];
extern const MaceCore::TopicComponentStructure Attitude_structure;

class AttitudeTopic : public DataState::Attitude, public Data::NamedTopicComponentDataObject<Attitude_name, &Attitude_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

}

#endif // ATTITUDE_TOPIC_H
