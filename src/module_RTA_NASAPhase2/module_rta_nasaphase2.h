#ifndef MODULE_RTA_NASAPHASE2_H
#define MODULE_RTA_NASAPHASE2_H

#include "module_rta_nasaphase2_global.h"

#include "mace_core/i_module_command_RTA.h"

class MODULE_RTA_NASAPHASE2SHARED_EXPORT ModuleRTANASAPhase2 : public MaceCore::IModuleCommandRTA
{

public:
    ModuleRTANASAPhase2();


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

public:

    virtual void NewVehicle(const std::string &ID);

    virtual void RemoveVehicle(const std::string &ID);

    virtual void UpdatedPositionDynamics(const std::string &vehicleID);

    virtual void UpdateAttitudeDynamics(const std::string &vehicleID);

    virtual void UpdatedVehicleLife(const std::string &vehicleID);

    virtual void UpdatedOccupancyMap();
};

#endif // MODULE_RTA_NASAPHASE2_H
