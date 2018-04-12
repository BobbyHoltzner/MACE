#ifndef TOPIC_COMPONENT_VECTOR_H
#define TOPIC_COMPONENT_VECTOR_H


#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{


extern const char TopicComponts_Vector_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_Vector_structure;

template <typename T>
class Vector : public Data::NamedTopicComponentDataObject<TopicComponts_Vector_name, &TopicComponts_Vector_structure>
{
private:

    std::vector<T> m_Arr;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const
    {
        MaceCore::TopicDatagram diagram;
        diagram.AddTerminal<T>("string", m_Arr);
        return diagram;
    }

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
    {

    }

    Vector();

    Vector(const std::vector<T> &arr);

    size_t Size()
    {
        return m_Arr.size();
    }

    std::vector<T>::const_reference at(const size_t i) const
    {
        return m_Arr.at(i);
    }
};


}

} // BaseTopics


#endif // TOPIC_COMPONENT_VECTOR_H
