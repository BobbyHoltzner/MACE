#ifndef TOPIC_COMPONENT_VECTOR_H
#define TOPIC_COMPONENT_VECTOR_H


#include "data/i_topic_component_data_object.h"

namespace Data {

namespace TopicComponents
{

template <typename T>
class Vector : public ITopicComponentDataObject
{
private:

    static constexpr MaceCore::TopicComponentStructure structure = []{
        MaceCore::TopicComponentStructure subStructure = *(T::Structure());

        MaceCore::TopicComponentStructure a;
        a.AddNonTerminal(std::string("arr"), subStructure);
        return a;
    }();

    std::vector<T> m_Arr;

public:

    static std::string Name() {
        return std::string("arr_") + std::string(T::Name());
    }

    static MaceCore::TopicComponentStructure* TopicStructure() {
        return &structure;
    }

    virtual MaceCore::TopicDatagram GenerateDatagram() const
    {
        MaceCore::TopicDatagram datagram;
        for(size_t i = 0 ; i < m_Arr.size() ; i++)
        {
            datagram.AddNonTerminal("arr", i, m_Arr.at(i).GenerateDatagram());
        }
        return datagram;
    }

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
    {
        throw std::runtime_error("Not Implimented");
    }

    Vector()
    {

    }

    Vector(const std::vector<T> &arr)
    {
        m_Arr = arr;
    }

    size_t Size()
    {
        return m_Arr.size();
    }


    typename std::vector<T>::reference operator[](const size_t i)
    {
        return m_Arr[i];
    }

    typename std::vector<T>::const_reference at(const size_t i) const
    {
        return m_Arr.at(i);
    }
};


}

} // BaseTopics


#endif // TOPIC_COMPONENT_VECTOR_H
