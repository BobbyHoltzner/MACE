#ifndef TOPIC_COLLECTION_H
#define TOPIC_COLLECTION_H

#include "topic.h"

#include <functional>
#include "mace_core/topic.h"
#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"

namespace MaceCore
{



template <const char* N, typename T>
class NamedTopic
{
private:

    T m_obj;

    OptionalParameter<std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)>> m_ConsumeAction;

public:

    NamedTopic() :
        m_obj(N)
    {

    }


    static const char* Name()
    {
        return N;
    }


    T* Get() {
        return Get<N>();
    }

    template <const char* NN, typename = typename std::enable_if< N == NN >::type >
    T* Get() {
        return &m_obj;
    }







    void AddConsumptionAction(const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &func)
    {
        AddConsumptionAction<N>(func);
    }

    template <const char* NN, typename = typename std::enable_if< N == NN >::type >
    T* AddConsumptionAction(const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &func)
    {
        m_ConsumeAction = func;
        return &m_obj;
    }



    void ConsumeAction(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)
    {
        ConsumeAction<N>(sender, data, target);
    }

    template <const char* NN, typename = typename std::enable_if< N == NN >::type >
    void ConsumeAction(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)
    {

        if(m_ConsumeAction.IsSet() == false)
        {
            throw std::runtime_error("Consume not action not set for this topic!");
        }

        m_ConsumeAction.Value()(sender, data, target);
    }
};








template <typename ...T>
class TopicCollection;


template <typename HEAD, typename ...TAIL>
class TopicCollection<HEAD, TAIL...> :
        public TopicCollection<TAIL...>,
        public HEAD
{
private:

protected:

    using TopicCollection<TAIL...>::m_Getters;
    using TopicCollection<TAIL...>::m_AddConsumers;
    using TopicCollection<TAIL...>::m_Consumers;

public:

    using TopicCollection<TAIL...>::Get;
    using HEAD::Get;

    using TopicCollection<TAIL...>::ConsumeAction;
    using HEAD::ConsumeAction;

    using TopicCollection<TAIL...>::AddConsumptionAction;
    using HEAD::AddConsumptionAction;


    TopicCollection<HEAD, TAIL...>()
    {
        std::string name = HEAD::Name();

        m_Getters.insert({name, [this](){
            return HEAD::Get();
        }});

        m_AddConsumers.insert({name, [this](const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &lambda){
            HEAD::AddConsumptionAction(lambda);
        }});

        m_Consumers.insert({name, [this](const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target){
            HEAD::ConsumeAction(sender, data, target);
        }});
    }


    template<typename T>
    T* Get(const char* name)
    {
        return (T*)m_Getters.at(name)();
    }

    void AddConsumeAction(const char* name, const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &func)
    {
        m_AddConsumers.at(name)(func);
    }


    void ConsumeAction(const char *name, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)
    {
        m_Consumers.at(name)(sender, data, target);
    }

};

template <>
class TopicCollection<>
{
private:

protected:

    std::unordered_map<std::string, std::function<void*()>> m_Getters;
    std::unordered_map<std::string, std::function<void(const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &)>> m_AddConsumers;
    std::unordered_map<std::string, std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)>> m_Consumers;

public:

    void Get() {}

    void ConsumeAction() {}

    void AddConsumptionAction() {}

};









/*
// Primary template (does not define key_type)
template<typename T, bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION, typename = void, typename = void>
struct _ConsumptionBehavior
{

    void Consume(const T *ptr, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target) {
    }
};

// Specialization using SFINAE to check for the existence of key() const
// (does define key_type)
template<typename T, bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION>
struct _ConsumptionBehavior<
    T,
    TOPIC_CONSUMED,
    FORCE_COMPILETIME_IMPLIMENTATION,
    typename std::enable_if<TOPIC_CONSUMED>::type,
    typename std::enable_if<!FORCE_COMPILETIME_IMPLIMENTATION>::type
    >
{

private:

    OptionalParameter<std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)>> m_ConsumeAction;

public:

    void Consume(const T *ptr, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target) {
        if(m_ConsumeAction.IsSet() == false)
        {
            throw std::runtime_error("Consume not action not set for this topic!");
        }

        m_ConsumeAction.Value()(sender, data, target);
    }


    void AddConsumptionAction(const std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)> &func)
    {
        m_ConsumeAction = func;
    }

};


// Specialization using SFINAE to check for the existence of key() const
// (does define key_type)
template<typename T, bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION>
struct _ConsumptionBehavior<
    T,
    TOPIC_CONSUMED,
    FORCE_COMPILETIME_IMPLIMENTATION,
    typename std::enable_if<TOPIC_CONSUMED>::type,
    typename std::enable_if<FORCE_COMPILETIME_IMPLIMENTATION>::type
    >
{
    void Consume(const T *ptr, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target){
        TopicAction_Consume(ptr, sender, data, target);
    }

    virtual void TopicAction_Consume(const T *ptr, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target) = 0;
};





template <bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION, typename ...T>
class TopicCollection;


template <bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION, typename HEAD, typename ...TAIL>
class TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, HEAD, TAIL...> :
        public TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, TAIL...>,
        public _ConsumptionBehavior<HEAD, TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION>
{
public:

    using TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, TAIL...>::Get;

protected:
    using TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, TAIL...>::mTopics;
    using TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, TAIL...>::mConsumers;

public:

    HEAD m_obj;

public:
    TopicCollection(const std::vector<std::string> &vec) :
        TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION, TAIL...>(std::vector<std::string>( vec.begin() + 1, vec.end() )),
        m_obj(vec.at(0))
    {
        mTopics.insert({vec.at(0), &m_obj});
        mConsumers.insert({vec.at(0), [this](const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target){
                _ConsumptionBehavior<HEAD, TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION>::Consume(&m_obj, sender, data, target);
            }});
    }

    void Get(HEAD* &ptr)
    {
        ptr = &m_obj;
    }
};

template <bool TOPIC_CONSUMED, bool FORCE_COMPILETIME_IMPLIMENTATION>
class TopicCollection<TOPIC_CONSUMED, FORCE_COMPILETIME_IMPLIMENTATION>
{
public:
    TopicCollection(const std::vector<std::string> &vec)
    {
        assert(vec.size() == 0);
    }

    _Topic<>* GetBaseTopic(const std::string &name) const
    {
        return mTopics.at(name);
    }

    void Get() {
    }

protected:

    std::unordered_map<std::string, _Topic<>*> mTopics;

    std::unordered_map<std::string, std::function<void(const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target)>> mConsumers;

};
*/

}
#endif // TOPIC_COLLECTION_H
