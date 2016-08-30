#include "configuration_reader_xml.h"

#include "pugixml.hpp"
#include "mace_core/module_factory.h"


ConfigurationReader_XML::ConfigurationReader_XML(const MaceCore::ModuleFactory *factory) :
    m_Factory(factory)
{

}


//!
//! \brief Parse all Parameter tags from an XML Node
//! \param node XML node that contains "Parameter" tags
//! \param structure Structure that parsing is expecting for this node
//! \return Resulting ModuleParameterValue object
//!
static std::shared_ptr<MaceCore::ModuleParameterValue> ParseParameters(const pugi::xml_node &node, const std::shared_ptr<MaceCore::ModuleParameterStructure> structure, ConfigurationParseResult &result)
{
    std::shared_ptr<MaceCore::ModuleParameterValue> valueContainer = std::make_shared<MaceCore::ModuleParameterValue>();


    //prepare map of all expected paramters, to be set when they are seen
    std::unordered_map<std::string, bool> seenTerminals;
    std::unordered_map<std::string, bool> seenNonTerminals;
    for(std::string item : structure->getNonTerminalNames())
    {
        seenNonTerminals.insert({item, false});
    }
    for(std::string item : structure->getTerminalNames())
    {
        seenTerminals.insert({item, false});
    }


    //loop through all "Parameter" Tags
    for (pugi::xml_node parameter = node.child("Parameter"); parameter; parameter = parameter.next_sibling("Parameter"))
    {
        std::string parameterName = parameter.attribute("Name").as_string();

        if(structure->TerminalExists(parameterName) == true)
        {
            std::string terminalStringValue = std::string(parameter.child_value());
            valueContainer->AddTerminalValueFromString(parameterName, terminalStringValue, structure->getTerminalType(parameterName));

            seenTerminals[parameterName] = true;
        }
        else if(structure->NonTerminalExists(parameterName) == true)
        {
            valueContainer->AddNonTerminal(parameterName, ParseParameters(parameter, structure->getNonTerminalStructure(parameterName), result));

            seenNonTerminals[parameterName] = true;
        }
        else
        {
            //The paramter in XML file is not defined in the structure, set as warning and continue;
            result.warnings.push_back(parameterName + " paramater present in configuration does not associate to any expected nonterminal/terminal structure");
        }
    }


    //go through an check for any unused paramters
    for(auto it = seenTerminals.cbegin() ; it != seenTerminals.cend() ; ++it)
    {
        //entry not present, check if required, otherwise add default value
        if(it->second == false)
        {
            if(structure->IsTagRequired(it->first) == true)
            {
                result.error = it->first + " Parameter is not present and is marked as required";
                result.success = false;
                return valueContainer;
            }
            else
            {
                std::string defaultValue = structure->getDefaultTerminalValue(it->first);
                result.warnings.push_back(it->first + " Not set, using default value of " + defaultValue);
                valueContainer->AddTerminalValueFromString(it->first, defaultValue, structure->getTerminalType(it->first));
            }
        }
    }
    for(auto it = seenNonTerminals.cbegin() ; it != seenNonTerminals.cend() ; ++it)
    {
        //entry not present, check if required, otherwise add default value
        if(it->second == false)
        {
            if(structure->IsTagRequired(it->first) == true)
            {
                result.error = it->first + " Parameter is not present and is marked as required";
                result.success = false;
                return valueContainer;
            }
            else
            {
                result.warnings.push_back(it->first + " Not set, using default value");
                valueContainer->AddNonTerminal(it->first, structure->getDefaultNonTerminalValue(it->first));
            }
        }
    }


    return valueContainer;
}


//!
//! \brief Parse an given XML file
//!
//! What parser will be lookign for is dependent on what modules were added with AddModule
//! \param filename Filename to parse
//! \return True if parsing was success
//!
ConfigurationParseResult ConfigurationReader_XML::Parse(const std::string &filename)
{
    ConfigurationParseResult parseProgress;


    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if(!result)
        return ConfigurationParseResult("Not Valid XML File");


    //loop through each "Module" tag in the XML document under "ModuleConfigurations" tag
    pugi::xml_node moduleConfigurationsNode = doc.child("ModuleConfigurations");
    for (pugi::xml_node module = moduleConfigurationsNode.child("Module"); module; module = module.next_sibling("Module"))
    {
        //if module is disabled then skip parsing
        if(module.attribute("Enabled").empty() == false)
        {
            if(module.attribute("Enabled").as_bool() == false)
                continue;
        }

        //determine module class, error if unsuccessfull
        std::string moduleClassName = module.attribute("Class").as_string();
        MaceCore::ModuleBase::Classes moduleClass;
        try
        {
            moduleClass = MaceCore::ModuleBase::StringToModuleClass(moduleClassName);
        }
        catch(const std::runtime_error &e)
        {
            parseProgress.success = false;
            parseProgress.error = moduleClassName + " is not valid module class";
            return parseProgress;
        }


        //attempt to create module, error if unsuccesfull
        std::shared_ptr<MaceCore::ModuleBase> newModule;
        std::string moduleType = module.attribute("Type").as_string();
        try
        {
            newModule = m_Factory->Create(moduleClass, moduleType);
        }
        catch (const std::runtime_error &e)
        {
            parseProgress.success = false;
            parseProgress.error = "Unable to create module: " + std::string(e.what());
            return parseProgress;
        }


        //parse parameters
        std::shared_ptr<MaceCore::ModuleParameterStructure> structure = newModule->ModuleConfigurationStructure();
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleValue = ParseParameters(module, structure, parseProgress);


        //if not successfull return
        if(parseProgress.success == false)
            return parseProgress;


        //insert in map
        m_ModuleTypes.insert({newModule, moduleType});
        m_Parameters.insert({newModule, moduleValue});
    }


    return parseProgress;
}


//!
//! \brief Get modules created after parsing
//! \return List of created modules.
//!
std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> ConfigurationReader_XML::GetCreatedModules() const
{
    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> map;
    for(auto it = m_ModuleTypes.cbegin(); it != m_ModuleTypes.cend() ; ++it)
        map.insert({it->first, it->second});

    return map;
}


//!
//! \brief Get the configuration for a module after parse
//!
//! Must be called after Parse is called and returns with a value of true
//! \param module Pointer to module to get configuration of
//! \return Configuration for module
//!
std::shared_ptr<MaceCore::ModuleParameterValue> ConfigurationReader_XML::GetModuleConfiguration(const std::shared_ptr<MaceCore::ModuleBase> &module)
{
    return m_Parameters.at(module);
}
