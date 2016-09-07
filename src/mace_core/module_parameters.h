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
    STRING,
    BOOLEAN
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

    static bool FromString(const std::string &string, bool &value)
    {
        if(string == "true" || string == "True" || string == "TRUE")
        {
            value = true;
            return true;
        }
        if(string == "false" || string == "False" || string == "FALSE")
        {
            value = false;
            return true;
        }
        return false;
    }

};





//!
//! \brief Class that holds parameter values for a module
//!
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
        case ModuleParameterTerminalTypes::BOOLEAN:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<bool>(valueStr));
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

    void AddTerminalValue(const std::string &name, const bool &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<bool> >(SingleParameterValue<bool>(value))});;
    }

    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterValue> &value)
    {
        m_NonTerminalValues.insert({name, value});
    }


    //!
    //! \brief Determine if given terminal parameter exists
    //! \param name Parameter name
    //! \return True if exists
    //!
    bool HasTerminal(const std::string &name) const
    {
        //check that given parameter exists
        if(m_TerminalValues.find(name) == m_TerminalValues.cend())
            return false;
        return true;
    }


    //!
    //! \brief Determine if given non-terminal parameter exists
    //! \param name Parameter name
    //! \return True if exists
    //!
    bool HasNonTerminal(const std::string &name) const
    {
        if(m_NonTerminalValues.find(name) == m_NonTerminalValues.cend())
            return false;
        return true;
    }


    //!
    //! \brief Get value of a terminal parameter
    //! \param name Name of parameter
    //! \return Value
    //! \throws std::runtime_error Thrown if given terminal does not exists
    //!
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


    //!
    //! \brief Get the value of a non-terminal parameter
    //! \param name Name of non-terminal parameter
    //! \return Value
    //! \throws std::runtime_error Thrown if given non-terminal does not exists
    //!
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


//!
//! \brief Class that describes the structure of a parameters allowed by a module
//!
//! This class will be used to parse incomming settings and ensure the settings are of the form the module is expecting
//!
class ModuleParameterStructure
{
public:



    //!
    //! \brief Add a terminal to known parameters
    //!
    //! A non terminal paramter is an actuall piece of data to store as a setting to a module
    //! \param name Name of parameter
    //! \param type Data type expecting
    //! \param required True if value is required
    //! \param defaultValue Value to set if parameter is not required and not present.
    //! \param allowedEntires Descrete set of value that are stricktly allowed
    //!
    void AddTerminalParameters(const std::string &name, const ModuleParameterTerminalTypes &type, bool required = false, const std::string &defaultValue = "", const std::vector<std::string> &allowedEntires = {})
    {
        m_TerminalParams.insert({name, type});
        m_IsTagRequired.insert({name, required});
        m_TerminalDefaultValue.insert({name, defaultValue});
        m_TerminalAllowedEntries.insert({name, allowedEntires});
    }


    //!
    //! \brief Add a non terminal to known parameters
    //!
    //! A nonterminal is a "parameter set", it is a catatory which is expected to contain other terminals with actual data.
    //! \param name Name of non terminal parameter set
    //! \param type Expected format of non-terminal
    //! \param required True if required
    //! \param defaultValue Default value to set if not required and not present in settings
    //! \param multipleEntiresAllowed True if multiple entires of this non-terminal are allowed (not implimented)
    //!
    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterStructure> &type, bool required = false, const std::shared_ptr<ModuleParameterValue> &defaultValue = std::make_shared<ModuleParameterValue>(), bool multipleEntiresAllowed = false)
    {
        m_NonTerminalParams.insert({name, type});
        m_IsTagRequired.insert({name, required});
        m_NonTerminalDefaultValue.insert({name, defaultValue});
        m_NonTerminalMultipleAllowed.insert({name, multipleEntiresAllowed});
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


    //!
    //! \brief Given a pameter return the entires allowed for that parameter
    //! \param parameterName Name of parameter
    //! \return Descrite values allowed. Empty if unrestricted.
    //!
    std::vector<std::string> getTerminalAllowedEntires(const std::string &parameterName) const
    {
        if(m_TerminalAllowedEntries.find(parameterName) == m_TerminalAllowedEntries.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalAllowedEntries.at(parameterName);
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

    bool getNonTerminalMultipleEntriesAllowed(const std::string &parameterName) const
    {
        if(m_NonTerminalMultipleAllowed.find(parameterName) == m_NonTerminalMultipleAllowed.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalMultipleAllowed.at(parameterName);
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

    std::unordered_map<std::string, std::vector<std::string>> m_TerminalAllowedEntries;

    std::unordered_map<std::string, bool> m_IsTagRequired;

    std::unordered_map<std::string, ModuleParameterTerminalTypes> m_TerminalParams;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterStructure> > m_NonTerminalParams;

    std::unordered_map<std::string, bool> m_NonTerminalMultipleAllowed;
};

} //End MaceCore Namespace

#endif // MODULE_PARAMETERS_H
