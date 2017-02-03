#ifndef LOCAL_POSITION_TOPIC_H
#define LOCAL_POSITION_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_items/local_position.h"

namespace DataStateTopic
{

extern const char LocalPosition_name[];
extern const MaceCore::TopicComponentStructure LocalPosition_structure;

class LocalPositionTopic : public DataState::LocalPosition, public Data::NamedTopicComponentDataObject<LocalPosition_name, &LocalPosition_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    LocalPositionTopic() :
        DataState::LocalPosition()
    {

    }
};

}

#endif // DATAVEHICLEGENERIC_LOCALPOSITION_H
