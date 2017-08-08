#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>

#include "mace_core_global.h"

#include "mace_data.h"

#include "module_parameters.h"

#include "topic.h"

namespace MaceCore
{

//!
//! \brief Base abstract class for a Module
//!
class ModuleBase
{
public:
    //!
    //! \brief Types of modules that can be instantiated
    //!
    enum Classes
    {
        EXTERNAL_LINK,
        GROUND_STATION,
        PATH_PLANNING,
        RTA,
        SENSORS,
        VEHICLE_COMMS,
        NR_TYPES
    };

    static std::string ModuleTypeToString(const Classes &type)
    {
        switch (type) {
        case EXTERNAL_LINK:
            return "ExternalLink";
        case GROUND_STATION:
            return "GroundStation";
        case PATH_PLANNING:
            return "PathPlanning";
        case RTA:
            return "RTA";
        case SENSORS:
            return "Sensors";
        case VEHICLE_COMMS:
            return "VehicleComms";
        default:
            throw std::runtime_error("Unknown module type");
        }
    }


    static Classes StringToModuleClass(const std::string &string)
    {
        if(string == "ExternalLink")
            return EXTERNAL_LINK;
        if(string == "GroundStation")
            return GROUND_STATION;
        if(string == "PathPlanning")
            return PATH_PLANNING;
        if(string == "RTA")
            return RTA;
        if(string == "Sensors")
            return SENSORS;
        if(string == "VehicleComms")
            return VEHICLE_COMMS;
        throw std::runtime_error("Unknown module type");
    }

public:


    const static Classes moduleClass;

    virtual Classes ModuleClass() const = 0;


    //!
    //! \brief function that is to kick off the event loop of the module
    //!
    virtual void start() = 0;


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<ModuleParameterStructure> ModuleConfigurationStructure() const = 0;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<ModuleParameterValue> &params) = 0;

    //!
    //! \brief AssignLoggingDirectory
    //! \param path
    //!
    void AssignLoggingDirectory(const std::string &path)
    {
        this->loggingPath = path;
    }


    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated) = 0;


    virtual std::unordered_map<std::string, TopicStructure> GetTopics()
    {
        //TODO make pure
        return {};
    }


    void setDataObject(const std::shared_ptr<MaceData> &data)
    {
        m_Data = data;
    }

    std::shared_ptr<const MaceData> getDataObject() const
    {
        return m_Data;
    }

protected:
    std::string loggingPath;

private:
    std::shared_ptr<const MaceData> m_Data;
};


} //End MaceCore Namespace


#endif // MODULE_INTERFACE_H
