#ifndef I_TOPIC_COMPONENT_DATA_OBJECT_H
#define I_TOPIC_COMPONENT_DATA_OBJECT_H

#include "mace_core/topic.h"

namespace Data {

enum TopicType{
    VEHICLESTATE,
    VEHICLEMISSION
};

class ITopicComponentDataObject : public MaceCore::ITopicComponentPrototype {
public:
    const char* name;
};


template<const char* CompName, const MaceCore::TopicComponentStructure *Structure>
class NamedTopicComponentDataObject : public ITopicComponentDataObject{

public:
    NamedTopicComponentDataObject()
    {
        name = CompName;
    }

    static std::string Name() {
        return std::string(CompName);
    }

    static MaceCore::TopicComponentStructure* TopicStructure() {
        return Structure;
    }


};


}

#endif // I_TOPIC_COMPONENT_DATA_OBJECT_H
