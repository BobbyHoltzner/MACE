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



class ParameterConversion
{
public:


    template<typename T>
    static T ConvertFromString(const std::string &string)
    {
        T value;
        bool success = FromString(string, value);

        if(success == false)
            throw std::runtime_error("Error convering string");

        return value;
    }


private:


    static bool FromString(const std::string &string, int &value)
    {
        try
        {
            value = std::stoi (string);
            return true;
        }
        catch(const std::invalid_argument)
        {
            return false;
        }
    }

    static bool FromString(const std::string &string, double &value)
    {
        try
        {
            value = std::stod (string);
            return true;
        }
        catch(const std::invalid_argument)
        {
            return false;
        }
    }

    static bool FromString(const std::string &string, std::string &value)
    {
        value = string;
        return true;
    }

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

    void AddTerminalValueFromString(const std::string &name, const std::string &valueStr, const ModuleParameterTerminalTypes &type)
    {
        switch(type)
        {
        case ModuleParameterTerminalTypes::INT:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<int>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::DOUBLE:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<double>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::STRING:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<std::string>(valueStr));
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown type");
        }
        }
    }

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


class ModuleParameterStructure
{
public:



    void AddTerminalParameters(const std::string &name, const ModuleParameterTerminalTypes &type, bool required = false, const std::string &defaultValue = "")
    {
        m_TerminalParams.insert({name, type});
        m_IsTagRequired.insert({name, required});
        m_TerminalDefaultValue.insert({name, defaultValue});
    }

    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterStructure> &type, bool required = false, const std::shared_ptr<ModuleParameterValue> &defaultValue = std::make_shared<ModuleParameterValue>())
    {
        m_NonTerminalParams.insert({name, type});
        m_IsTagRequired.insert({name, required});
        m_NonTerminalDefaultValue.insert({name, defaultValue});
    }


    std::vector<std::string> getTerminalNames() const
    {
        std::vector<std::string> keys;
        for(auto it = m_TerminalParams.cbegin() ; it != m_TerminalParams.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }

    ModuleParameterTerminalTypes getTerminalType(const std::string &parameterName) const
    {
        if(m_TerminalParams.find(parameterName) == m_TerminalParams.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalParams.at(parameterName);
    }


    std::string getDefaultTerminalValue(const std::string &parameterName) const
    {
        if(m_TerminalDefaultValue.find(parameterName) == m_TerminalDefaultValue.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalDefaultValue.at(parameterName);
    }


    std::vector<std::string> getNonTerminalNames() const
    {
        std::vector<std::string> keys;
        for(auto it = m_NonTerminalParams.cbegin() ; it != m_NonTerminalParams.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }

    const std::shared_ptr<ModuleParameterStructure> getNonTerminalStructure(const std::string &parameterName) const
    {
        if(m_NonTerminalParams.find(parameterName) == m_NonTerminalParams.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalParams.at(parameterName);
    }


    std::shared_ptr<ModuleParameterValue> getDefaultNonTerminalValue(const std::string &parameterName) const
    {
        if(m_NonTerminalDefaultValue.find(parameterName) == m_NonTerminalDefaultValue.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalDefaultValue.at(parameterName);
    }


    //!
    //! \brief returns true if the given parameter name is a terminal
    //! \param paramName Name of parameter
    //! \return true is exists
    //!
    bool TerminalExists(const std::string &paramName) const
    {
        for(auto it = m_TerminalParams.cbegin() ; it != m_TerminalParams.cend() ; ++it)
        {
            if(it->first == paramName)
                return true;
        }

        return false;
    }


    //!
    //! \brief returns true if the given parameter name is a non terminal
    //! \param paramName Name of parameter
    //! \return true is exists
    //!
    bool NonTerminalExists(const std::string &paramName) const
    {
        for(auto it = m_NonTerminalParams.cbegin() ; it != m_NonTerminalParams.cend() ; ++it)
        {
            if(it->first == paramName)
                return true;
        }

        return false;
    }


    //!
    //! \brief Return if provided tag is required in the structure
    //! \param paramName Param to look for required status
    //! \return True if required
    //!
    bool IsTagRequired(const std::string &paramName) const
    {
        return m_IsTagRequired.at(paramName);
    }

private:

    std::unordered_map<std::string, std::string> m_TerminalDefaultValue;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterValue>> m_NonTerminalDefaultValue;

    std::unordered_map<std::string, bool> m_IsTagRequired;

    std::unordered_map<std::string, ModuleParameterTerminalTypes> m_TerminalParams;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterStructure> > m_NonTerminalParams;
};

} //End MaceCore Namespace

#endif // MODULE_PARAMETERS_H
