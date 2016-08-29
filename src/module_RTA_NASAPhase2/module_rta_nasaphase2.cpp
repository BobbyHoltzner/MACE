#include "module_rta_nasaphase2.h"

#include "mace_core/module_factory.h"

ModuleRTANASAPhase2::ModuleRTANASAPhase2() :
    MaceCore::IModuleCommandRTA()

{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleRTANASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTANASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

}


void ModuleRTANASAPhase2::NewVehicle(const std::string &ID)
{

}


void ModuleRTANASAPhase2::RemoveVehicle(const std::string &ID)
{

}


void ModuleRTANASAPhase2::UpdatedPositionDynamics(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdateAttitudeDynamics(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdatedVehicleLife(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdatedOccupancyMap()
{

}
