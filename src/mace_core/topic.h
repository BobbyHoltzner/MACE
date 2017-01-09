#ifndef TOPIC_H
#define TOPIC_H

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

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

private:

    std::unordered_map<std::string, std::string> m_TerminalDataFields;
    std::unordered_map<std::string, std::shared_ptr<TopicComponentStructure>> m_NonTerminalDataFields;
};


class TopicStructure
{
public:

    TopicStructure()
    {

    }

    AddComponent(const std::string &name, const TopicComponentStructure &datagram, bool required = true){
        m_Components.insert({name, datagram});
        m_ComponentRequired.insert({name, required});
    }

private:
    std::unordered_map<std::string, TopicComponentStructure> m_Components;
    std::unordered_map<std::string, bool> m_ComponentRequired;
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

    template<typename T>
    void AddTerminal(const std::string &str, const T &value){
        std::shared_ptr<SingleParameterValue<T>> ptr = std::make_shared<SingleParameterValue<T>>(value);
        m_TerminalValues.insert({str, ptr});
    }

    template<typename T>
    T GetTerminal(const std::string &str) {
        if(m_TerminalValues.find(str) == m_TerminalValues.cend()) {
            throw std::runtime_error("Given string does not exists as a value in this topic data");
        }

        std::shared_ptr<void> void_ptr = m_TerminalValues.at(str);

        std::shared_ptr<SingleParameterValue<T>> ptr = std::dynamic_pointer_cast<SingleParameterValue<T>>(void_ptr);
        if(ptr == NULL){
            throw std::runtime_error("Given value type does not match expected type");
        }

        return ptr->GetValue();
    }


    void AddNonTerminal(const std::string &str, const TopicDatagram &values) {
        std::shared_ptr<TopicDatagram> ptr = std::make_shared<TopicDatagram>(values);
        m_NonTerminalValues.insert({str, ptr});
    }

    std::shared_ptr<TopicDatagram> GetNonTerminal(const std::string &str){
        return m_NonTerminalValues.at(str);
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

private:

    std::unordered_map<std::string, std::shared_ptr<void> > m_TerminalValues;
    std::unordered_map<std::string, std::shared_ptr<TopicDatagram> > m_NonTerminalValues;
};

}

#endif // TOPIC_H
