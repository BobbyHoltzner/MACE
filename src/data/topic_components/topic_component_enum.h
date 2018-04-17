#ifndef TOPIC_COMPONENT_ENUM_H
#define TOPIC_COMPONENT_ENUM_H

#include "common/optional_parameter.h"
#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{

extern const char TopicComponts_Enum_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_Enum_structure;

template <typename E>
class Enum : public Data::NamedTopicComponentDataObject<TopicComponts_Enum_name, &TopicComponts_Enum_structure>
{
private:
    OptionalParameter<E> m_Enum;

public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const
    {
        MaceCore::TopicDatagram diagram;
        diagram.AddTerminal("enum", m_Enum);
        return diagram;
    }

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
    {
        int integer = datagram.GetTerminal<int>("enum");
        m_Enum = integer;
    }

    Enum()
    {

    }

    Enum(const E value)
    {
        m_Enum = value;
    }

    E Value() const
    {
        return m_Enum();
    }

};

}

}

#endif // TOPIC_COMPONENT_ENUM_H
