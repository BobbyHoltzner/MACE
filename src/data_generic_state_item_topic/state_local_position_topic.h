#ifndef STATE_LOCAL_POSITION_TOPIC_H
#define STATE_LOCAL_POSITION_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_local_position.h"

namespace DataStateTopic {

extern const char LocalPositionTopic_name[];
extern const MaceCore::TopicComponentStructure LocalPositionTopic_structure;

class StateLocalPositionTopic : public DataState::StateLocalPosition, public Data::NamedTopicComponentDataObject<LocalPositionTopic_name, &LocalPositionTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    StateLocalPositionTopic();
};

} //end of namespace DataStateTopic

#endif // STATE_LOCAL_POSITION_TOPIC_H
