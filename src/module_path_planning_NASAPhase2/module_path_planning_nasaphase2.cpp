#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>

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

void ModulePathPlanningNASAPhase2::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID)
{

}

