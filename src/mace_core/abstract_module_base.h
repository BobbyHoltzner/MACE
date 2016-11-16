#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>

#include "mace_core_global.h"

#include "mace_data.h"

#include "module_parameters.h"

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
        PATH_PLANNING,
        RTA,
        VEHICLE_COMMS,
        GROUND_STATION,
        NR_TYPES
    };

    static std::string ModuleTypeToString(const Classes &type)
    {
        switch (type) {
        case VEHICLE_COMMS:
            return "VehicleComms";
        case RTA:
            return "RTA";
        case PATH_PLANNING:
            return "PathPlanning";
        case GROUND_STATION:
            return "GroundStation";
        default:
            throw std::runtime_error("Unknown module type");
        }
    }


    static Classes StringToModuleClass(const std::string &string)
    {
        if(string == "VehicleComms")
            return VEHICLE_COMMS;
        if(string == "RTA")
            return RTA;
        if(string == "PathPlanning")
            return PATH_PLANNING;
        if(string == "GroundStation")
            return GROUND_STATION;

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


    void setDataObject(const std::shared_ptr<MaceData> &data)
    {
        m_Data = data;
    }

    std::shared_ptr<const MaceData> getDataObject() const
    {
        return m_Data;
    }

private:

    std::shared_ptr<const MaceData> m_Data;
};


} //End MaceCore Namespace


#endif // MODULE_INTERFACE_H
