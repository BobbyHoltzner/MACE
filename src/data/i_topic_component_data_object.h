#ifndef I_TOPIC_COMPONENT_DATA_OBJECT_H
#define I_TOPIC_COMPONENT_DATA_OBJECT_H

#include "mace_core/topic.h"

namespace Data {

class ITopicComponentDataObject{

public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const = 0;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) = 0;

};

}

#endif // I_TOPIC_COMPONENT_DATA_OBJECT_H
