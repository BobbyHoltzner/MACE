#ifndef TOPIC_H
#define TOPIC_H

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

namespace MaceCore
{


class TopicDatagramStructure
{
public:


    TopicDatagramStructure()
    {

    }

    template <typename T>
    void AddTerminal(const std::string &str)
    {

        m_TerminalDataFields.insert({str, typeid(T).name()});
    }

    void AddNonTerminal(const std::string &str, const TopicDatagramStructure &fields)
    {
        m_NonTerminalDataFields.insert({str, std::make_shared<TopicDatagramStructure>(fields)});
    }

private:

    std::unordered_map<std::string, std::string> m_TerminalDataFields;
    std::unordered_map<std::string, std::shared_ptr<TopicDatagramStructure>> m_NonTerminalDataFields;
};


class Topic
{
public:

    Topic()
    {

    }

    AddDatagram(const std::string &name, const TopicDatagramStructure &datagram, bool required = true){
        m_Datagrams.insert({name, datagram});
        m_DatagramsRequired.insert({name, required});
    }

private:
    std::unordered_map<std::string, TopicDatagramStructure> m_Datagrams;
    std::unordered_map<std::string, bool> m_DatagramsRequired;
};






class TopicValues
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


    void AddNonTerminal(const std::string &str, const TopicValues &values) {
        std::shared_ptr<TopicValues> ptr = std::make_shared<TopicValues>(values);
        m_NonTerminalValues.insert({str, ptr});
    }

    std::shared_ptr<TopicValues> GetNonTerminal(const std::string &str){
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

private:

    std::unordered_map<std::string, std::shared_ptr<void> > m_TerminalValues;
    std::unordered_map<std::string, std::shared_ptr<TopicValues> > m_NonTerminalValues;
};

}

#endif // TOPIC_H
