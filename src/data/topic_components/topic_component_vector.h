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

    static MaceCore::TopicCharacteristic *structure;

    std::vector<T> m_Arr;

public:

    static std::string Name() {
        return std::string("arr_") + std::string(T::Name());
    }

    static MaceCore::TopicComponentStructure TopicStructure() {
        MaceCore::TopicComponentStructure subStructure = *(T::Structure());

        MaceCore::TopicComponentStructure a;
        a.AddNonTerminal(std::string("arr"), subStructure);
        return MaceCore::TopicComponentStructure();
    }

    virtual MaceCore::TopicDatagram GenerateDatagram() const
    {
        MaceCore::TopicDatagram datagram;
        for(size_t i = 0 ; i < m_Arr.size() ; i++)
        {
            std::shared_ptr<MaceCore::TopicDatagram> ptr = std::make_shared<MaceCore::TopicDatagram>(m_Arr.at(i).GenerateDatagram());
            datagram.AddNonTerminal("arr", i, ptr);
        }
        return datagram;
    }

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
    {
        for(size_t i = 0 ; i < datagram.GetNonTerminal_Num("arr") ; i++)
        {
            std::shared_ptr<MaceCore::TopicDatagram> subData = datagram.GetNonTerminal_Index("arr", i);
            std::shared_ptr<T> ptr = std::make_shared<T>();
            ptr->CreateFromDatagram(*subData);
            m_Arr.push_back(*ptr);
        }
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

    void push_back(const T &obj)
    {
        m_Arr.push_back(obj);
    }

    typename std::vector<T>::const_reference at(const size_t i) const
    {
        return m_Arr.at(i);
    }
};


}

} // BaseTopics


#endif // TOPIC_COMPONENT_VECTOR_H
