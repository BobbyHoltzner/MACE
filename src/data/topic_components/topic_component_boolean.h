#ifndef TOPIC_COMPONENT_BOOLEAN_H
#define TOPIC_COMPONENT_BOOLEAN_H


#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{


extern const char TopicComponts_Boolean_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_Boolean_structure;

class Boolean : public Data::NamedTopicComponentDataObject<TopicComponts_Boolean_name, &TopicComponts_Boolean_structure>
{
private:

    bool m_value;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    Boolean();

    Boolean(const bool value);

    bool Value() const
    {
        return m_value;
    }
};


}

} // BaseTopics

#endif // TOPIC_COMPONENT_BOOLEAN_H
