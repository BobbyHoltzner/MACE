#ifndef MODULE_PARAMETERS_H
#define MODULE_PARAMETERS_H

#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <memory>

namespace MaceCore
{

enum class ModuleParameterTerminalTypes
{
    INT,
    DOUBLE,
    STRING
};


class ModuleParameterStructure
{
public:



    void AddTerminalParameters(const std::string &name, const ModuleParameterTerminalTypes &type)
    {
        m_TerminalParams.insert({name, type});
    }

    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterStructure> &type)
    {
        m_NonTerminalParams.insert({name, type});
    }

private:


    std::unordered_map<std::string, ModuleParameterTerminalTypes> m_TerminalParams;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterStructure> > m_NonTerminalParams;
};



class ModuleParameterValue
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

    void AddTerminalValue(const std::string &name, const int &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<int> >(SingleParameterValue<int>(value))});
    }

    void AddTerminalValue(const std::string &name, const double &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<double> >(SingleParameterValue<double>(value))});
    }

    void AddTerminalValue(const std::string &name, const std::string &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<std::string> >(SingleParameterValue<std::string>(value))});;
    }

    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterValue> &value)
    {
        m_NonTerminalValues.insert({name, value});
    }



    template<typename T>
    T GetTerminalValue(const std::string &name) const
    {
        //check that given parameter exists
        if(m_TerminalValues.find(name) == m_TerminalValues.cend())
            throw std::runtime_error("Not a terminal parameter value");

        const std::shared_ptr<void> basePtr = m_TerminalValues.at(name);

        const SingleParameterValue<T>* ptr = (const SingleParameterValue<T>*)basePtr.get();

        //check that types are correct
        if(ptr->GetType() != typeid(T))
            throw std::runtime_error("Type Missmatch");

        return ptr->GetValue();
    }

    std::shared_ptr<ModuleParameterValue> GetNonTerminalValue(const std::string &name) const
    {
        if(m_NonTerminalValues.find(name) == m_NonTerminalValues.cend())
            throw std::runtime_error("Not a non-terminal parameter value");

        return m_NonTerminalValues.at(name);
    }


private:

    std::unordered_map<std::string, std::shared_ptr<void> > m_TerminalValues;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterValue> > m_NonTerminalValues;
};

} //End MaceCore Namespace

#endif // MODULE_PARAMETERS_H
