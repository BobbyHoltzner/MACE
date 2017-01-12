#ifndef TOPIC_COMPONENT_COLLECTION_H
#define TOPIC_COMPONENT_COLLECTION_H

#include <functional>
#include <unordered_map>
#include <memory>


#include "i_topic_component_data_object.h"

namespace Data {

template <typename... Args>
class TopicDataObjectCollection;

template <>
class TopicDataObjectCollection<>
{
public:
    TopicDataObjectCollection(const std::string &name) :
        m_TopicName(name)
    {

    }


    MaceCore::TopicDatagram GenerateDatagram(const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &list) const
    {

    }


    std::string Name() {
        return m_TopicName;
    }



protected:
    std::string m_TopicName;
};



template <typename T, typename... Args>
class TopicDataObjectCollection<T, Args...> : public TopicDataObjectCollection<T>, public TopicDataObjectCollection<Args...>
{
public:
    TopicDataObjectCollection(const std::string &topicName) :
        TopicDataObjectCollection<T>(topicName),
        TopicDataObjectCollection<Args...>(topicName),
        m_TopicName(topicName)
    {
    }

    std::string Name() const {
        return m_TopicName;
    }

private:
    std::string m_TopicName;

};

template <typename T>
class TopicDataObjectCollection<T> : public TopicDataObjectCollection<>
{

public:
    TopicDataObjectCollection<T>(const std::string &topicName) :
        TopicDataObjectCollection<>(topicName)
    {
    }



    MaceCore::TopicStructure GetTopicStructure() {

        /*
        auto topic = TopicDataObjectCollection<T>::GetTopicStructure();

        topic.AddComponent(T::Name(), T::TopicStructure(), false);

        return topic;
        */
    }


    void SetComponent(const std::shared_ptr<T> &component, MaceCore::TopicDatagram *datagram) const {
        datagram->AddNonTerminal(T::Name(), component->GenerateDatagram());
    }


    std::shared_ptr<T> GetComponent(const MaceCore::TopicDatagram &datagram) const{
        if(datagram.HasNonTerminal(T::Name()) == false) {
            return std::shared_ptr<T>(NULL);
        }
        std::shared_ptr<T> ptr = std::make_shared<T>();
        ptr->CreateFromDatagram(*datagram.GetNonTerminal(T::Name()));
        return ptr;
    }
};

}

#endif // TOPIC_COMPONENT_COLLECTION_H
