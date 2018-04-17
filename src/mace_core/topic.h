#ifndef TOPIC_H
#define TOPIC_H

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "common/common.h"

namespace MaceCore
{


class TopicComponentStructure
{
public:


    TopicComponentStructure()
    {

    }

    template <typename T>
    void AddTerminal(const std::string &str)
    {

        m_TerminalDataFields.insert({str, typeid(T).name()});
    }

    void AddNonTerminal(const std::string &str, const TopicComponentStructure &fields)
    {
        m_NonTerminalDataFields.insert({str, std::make_shared<TopicComponentStructure>(fields)});
    }

    static TopicComponentStructure Merge(const TopicComponentStructure &merge1, const TopicComponentStructure &merge2)
    {
        TopicComponentStructure newStructure;

        for(auto it = merge1.m_TerminalDataFields.cbegin() ; it != merge1.m_TerminalDataFields.cend() ; ++it)
        {
            newStructure.m_TerminalDataFields.insert({it->first, it->second});
        }
        for(auto it = merge2.m_TerminalDataFields.cbegin() ; it != merge2.m_TerminalDataFields.cend() ; ++it)
        {
            newStructure.m_TerminalDataFields.insert({it->first, it->second});
        }


        for(auto it = merge1.m_NonTerminalDataFields.cbegin() ; it != merge1.m_NonTerminalDataFields.cend() ; ++it)
        {
            newStructure.m_NonTerminalDataFields.insert({it->first, it->second});
        }
        for(auto it = merge2.m_NonTerminalDataFields.cbegin() ; it != merge2.m_NonTerminalDataFields.cend() ; ++it)
        {
            newStructure.m_NonTerminalDataFields.insert({it->first, it->second});
        }

        return newStructure;
    }

private:

    std::unordered_map<std::string, std::string> m_TerminalDataFields;
    std::unordered_map<std::string, std::shared_ptr<TopicComponentStructure>> m_NonTerminalDataFields;
};









class TopicDatagram
{
private:
    template<typename T>
    class SingleParameterValue
    {
    public:

        SingleParameterValue(const T &value) :
            m_Type(typeid(T))
        {
            m_Value = value;
        }

        T GetValue() const
        {
            return m_Value;
        }

        const std::type_info& GetType() const
        {
            return m_Type;
        }

    private:

        const std::type_info& m_Type;
        T m_Value;
    };

public:

    static TopicDatagram Merge(const TopicDatagram &merge1, const TopicDatagram &merge2)
    {
        TopicDatagram newStructure;

        for(auto it = merge1.m_TerminalValues.cbegin() ; it != merge1.m_TerminalValues.cend() ; ++it)
        {
            newStructure.m_TerminalValues.insert({it->first, it->second});
        }
        for(auto it = merge2.m_TerminalValues.cbegin() ; it != merge2.m_TerminalValues.cend() ; ++it)
        {
            newStructure.m_TerminalValues.insert({it->first, it->second});
        }


        for(auto it = merge1.m_NonTerminalValues.cbegin() ; it != merge1.m_NonTerminalValues.cend() ; ++it)
        {
            newStructure.m_NonTerminalValues.insert({it->first, it->second});
        }
        for(auto it = merge2.m_NonTerminalValues.cbegin() ; it != merge2.m_NonTerminalValues.cend() ; ++it)
        {
            newStructure.m_NonTerminalValues.insert({it->first, it->second});
        }

        return newStructure;
    }

    template<typename T>
    void AddTerminal(const std::string &str, const T &value){
        std::shared_ptr<SingleParameterValue<T>> ptr = std::make_shared<SingleParameterValue<T>>(value);
        m_TerminalValues.insert({str, ptr});
    }

    template<typename T>
    T GetTerminal(const std::string &str) const {
        if(m_TerminalValues.find(str) == m_TerminalValues.cend()) {
            throw std::runtime_error("Given string does not exists as a value in this topic data");
        }

        std::shared_ptr<void> void_ptr = m_TerminalValues.at(str);

        const SingleParameterValue<T>* ptr = (const SingleParameterValue<T>*)void_ptr.get();
        if(ptr == NULL){
            throw std::runtime_error("Given value type does not match expected type");
        }

        return ptr->GetValue();
    }


    void AddNonTerminal(const std::string &str, const TopicDatagram &values) {
        std::shared_ptr<TopicDatagram> ptr = std::make_shared<TopicDatagram>(values);
        m_NonTerminalValues.insert({str, ptr});
    }

    void AddNonTerminal(const std::string &str, const std::shared_ptr<TopicDatagram> &values) {
        m_NonTerminalValues.insert({str, values});
    }

    void AddNonTerminal(const std::string &str, const int index, const std::shared_ptr<TopicDatagram> &values) {
        throw std::runtime_error("Non Implimented");
        //m_NonTerminalValues.insert({str, values});
    }

    std::shared_ptr<TopicDatagram> GetNonTerminal(const std::string &str) const {
        return m_NonTerminalValues.at(str);
    }

    bool HasNonTerminal(const std::string &str) const {
        if(m_NonTerminalValues.find(str) == m_NonTerminalValues.cend()){
            return false;
        }
        return true;
    }

    std::vector<std::string> ListTerminals() const{
        std::vector<std::string> keys;
        for(auto it = m_TerminalValues.cbegin() ; it != m_TerminalValues.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }


    std::vector<std::string> ListNonTerminals() const{
        std::vector<std::string> keys;
        for(auto it = m_NonTerminalValues.cbegin() ; it != m_NonTerminalValues.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }

    void MergeDatagram(const TopicDatagram &dataToMerge){
        for(auto it = dataToMerge.m_TerminalValues.cbegin() ; it != dataToMerge.m_TerminalValues.cend() ; ++it) {
            if(m_TerminalValues.find(it->first) == m_TerminalValues.cend()) {
                this->m_TerminalValues.insert({it->first, it->second});
            }
            else {
                this->m_TerminalValues[it->first] = it->second;
            }
        }
        for(auto it = dataToMerge.m_NonTerminalValues.cbegin() ; it != dataToMerge.m_NonTerminalValues.cend() ; ++it) {
            if(m_NonTerminalValues.find(it->first) == m_NonTerminalValues.cend()) {
                this->m_NonTerminalValues.insert({it->first, it->second});
            }
            else {
                this->m_NonTerminalValues[it->first] = it->second;
            }
        }
    }

    bool isEmpty() {
        if(m_TerminalValues.size() == 0 && m_NonTerminalValues.size() == 0)
            return true;
        return false;
    }

private:

    std::unordered_map<std::string, std::shared_ptr<void> > m_TerminalValues;
    std::unordered_map<std::string, std::shared_ptr<TopicDatagram> > m_NonTerminalValues;
};



class ITopicComponentPrototype {
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const = 0;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) = 0;
};


class TopicCharacteristic
{
private:
    bool m_Spooled;
    std::string m_Name;

public:

    TopicCharacteristic(bool spooled, const std::string &name) :
        m_Spooled(spooled),
        m_Name(name)
    {

    }

    bool Spooled() const
    {
        return m_Spooled;
    }

    std::string Name() const
    {
        return m_Name;
    }
};




template <typename... Args>
class _Topic;

template <>
class _Topic<>
{
public:
    _Topic(const std::string &name) :
        m_TopicName(name)
    {

    }

//TODO: Kenny commented this out as it was unused
//    MaceCore::TopicDatagram GenerateDatagram(const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &list) const
//    {
//        UNUSED(list);
//    }


    void SetComponent(std::shared_ptr<ITopicComponentPrototype> ptr, MaceCore::TopicDatagram &datagram) const {
        //TODO: WOULD PREFER THIS TO BE THROWN ON COMPILE INSTEAD OF RUNTIME
        throw std::runtime_error("Unknown component passed to topic");
    }


    bool GetComponent(std::shared_ptr<ITopicComponentPrototype> ptr, const MaceCore::TopicDatagram &datagram) const {
        //TODO: WOULD PREFER THIS TO BE THROWN ON COMPILE INSTEAD OF RUNTIME
        throw std::runtime_error("Unknown component passed to topic");
    }


    std::string Name() {
        return m_TopicName;
    }


    virtual TopicCharacteristic Characteristic() const = 0;


protected:
    std::string m_TopicName;
};



template <typename T, typename... Args>
class _Topic<T, Args...> : public _Topic<Args...>
{
public:
    //using _Topic<Args...>::SetComponent;
    //using _Topic<Args...>::GetComponent;

    _Topic(const std::string &topicName) :
        _Topic<Args...>(topicName),
        m_TopicName(topicName)
    {
    }

    std::string Name() const {
        return m_TopicName;
    }


    void SetComponent(std::shared_ptr<ITopicComponentPrototype> ptr, MaceCore::TopicDatagram &datagram) const {
        if(std::dynamic_pointer_cast<T>(ptr) != 0) {
            SetComponent(std::dynamic_pointer_cast<T>(ptr), datagram);
        }
        else {
            _Topic<Args...>::SetComponent(ptr, datagram);
        }
    }


    bool GetComponent(std::shared_ptr<ITopicComponentPrototype> ptr, const MaceCore::TopicDatagram &datagram) const {
        if(std::dynamic_pointer_cast<T>(ptr) != 0) {
            return GetComponent(datagram, std::dynamic_pointer_cast<T>(ptr));
        }
        else {
            return _Topic<Args...>::GetComponent(ptr, datagram);
        }

    }


    void SetComponent(const std::shared_ptr<T> &component, MaceCore::TopicDatagram &datagram) const {
        MaceCore::TopicDatagram dg = component->GenerateDatagram();
        datagram.AddNonTerminal(T::Name(), dg);
    }


    bool GetComponent(const MaceCore::TopicDatagram &datagram, std::shared_ptr<T> value) const{
        if(datagram.HasNonTerminal(T::Name()) == false) {
            return false;
        }

        value->CreateFromDatagram(*datagram.GetNonTerminal(T::Name()));
        return true;
    }

protected:
    std::string m_TopicName;

};



template <typename ...T>
class SpooledTopic : public _Topic<T...>
{
private:

public:
    SpooledTopic(std::string name) :
        _Topic<T...>(name)
    {
    }

    bool IsSpooled() const {
        return true;
    }

    virtual TopicCharacteristic Characteristic() const
    {
        return TopicCharacteristic(IsSpooled(), _Topic<T...>::m_TopicName);
    }


};

template <typename ...T>
class  NonSpooledTopic : public _Topic<T...>
{
private:

public:
    NonSpooledTopic(std::string name) :
        _Topic<T...>(name)
    {
    }

    bool IsSpooled() const {
        return false;
    }

    std::string Name() const {
        return _Topic<T...>::m_TopicName;
    }

    virtual TopicCharacteristic Characteristic() const
    {
        return TopicCharacteristic(IsSpooled(), _Topic<T...>::m_TopicName);
    }


};




























}

#endif // TOPIC_H
