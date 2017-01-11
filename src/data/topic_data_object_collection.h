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



protected:
    std::unordered_map<std::string, std::function<std::shared_ptr<ITopicComponentDataObject>(const MaceCore::TopicDatagram &datagram)>> m_Factory;
    std::string m_TopicName;
};



template <typename T, typename... Args>
class TopicDataObjectCollection<T, Args...> : public TopicDataObjectCollection<Args...>
{

public:
    TopicDataObjectCollection(const std::string &topicName) :
        TopicDataObjectCollection<Args...>(topicName)
    {
        this->m_Factory.insert({T::Name(), [](const MaceCore::TopicDatagram &datagram){
                                    if(datagram.HasNonTerminal(T::Name()) == false) {
                                        return std::shared_ptr<T>(NULL);
                                    }
                                    std::shared_ptr<T> ptr = std::make_shared<T>();
                                    ptr->CreateFromDatagram(*datagram.GetNonTerminal(T::Name()));
                                    return ptr;
                                }});
    }

    MaceCore::TopicStructure GetTopicStructure() {

        auto topic = TopicDataObjectCollection<Args...>::GetTopicStructure();

        topic.AddComponent(T::Name(), T::TopicStructure(), false);

        return topic;
    }

    std::shared_ptr<T> GetComponent(const MaceCore::TopicDatagram &datagram) const{
        return std::dynamic_pointer_cast<T>(this->m_Factory.at(T::Name())(datagram));
    }
};

}

#endif // TOPIC_COMPONENT_COLLECTION_H
