#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H

#include "module_path_planning_nasaphase2_global.h"

#include "mace_core/i_module_command_path_planning.h"


class ModulePathPlanningNASAPhase2 : public MaceCore::IModuleCommandPathPlanning
{

public:
    ModulePathPlanningNASAPhase2();

public:

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


    //!
    //! \brief function that is to kick off the path planning event loop
    //!
    virtual void start();




public:


    virtual void NewVehicle(const std::string &ID, const MaceCore::MetadataVehicle &vehicle);

    virtual void RemoveVehicle(const std::string &ID);

    virtual void UpdatedPosition(const std::string &vehicleID);

    virtual void UpdateDynamicsState(const std::string &vehicleID);

    virtual void UpdatedVehicleLife(const std::string &vehicleID);


    //!
    //! \brief New targets have been assigned to the given vehicle
    //! \param vehicleID ID of vehicle
    //!
    virtual void NewVehicleTarget(const std::string &vehicleID);
};

#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
