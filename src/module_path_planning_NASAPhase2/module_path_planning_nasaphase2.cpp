#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning()
{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

}


//!
//! \brief function that is to kick off the path planning event loop
//!
void ModulePathPlanningNASAPhase2::start()
{
    while(true)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}


void ModulePathPlanningNASAPhase2::NewVehicle(const std::string &ID, const MaceCore::MetadataVehicle &vehicle)
{

}


void ModulePathPlanningNASAPhase2::RemoveVehicle(const std::string &ID)
{

}


void ModulePathPlanningNASAPhase2::UpdatedPosition(const std::string &vehicleID)
{

}


void ModulePathPlanningNASAPhase2::UpdateDynamicsState(const std::string &vehicleID)
{

}


void ModulePathPlanningNASAPhase2::UpdatedVehicleLife(const std::string &vehicleID)
{

}


//!
//! \brief New targets have been assigned to the given vehicle
//! \param vehicleID ID of vehicle
//!
void ModulePathPlanningNASAPhase2::NewVehicleTarget(const std::string &vehicleID)
{

}
