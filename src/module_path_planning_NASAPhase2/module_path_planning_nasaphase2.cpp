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


void ModulePathPlanningNASAPhase2::NewVehicle(const std::string &ID)
{

}


void ModulePathPlanningNASAPhase2::RemoveVehicle(const std::string &ID)
{

}


void ModulePathPlanningNASAPhase2::UpdatedPositionDynamics(const std::string &vehicleID)
{
    std::shared_ptr<MaceCore::MaceData> data = this->getDataObject();

    MaceCore::TIME time;
    //get current time

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    data->GetPositionDynamics(vehicleID, time, pos, vel);

    //do something with pos and vel
}


void ModulePathPlanningNASAPhase2::UpdateAttitudeDynamics(const std::string &vehicleID)
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


//!
//! \brief For one reason or another a recomputation of all vehicles' paths is requested
//!
void ModulePathPlanningNASAPhase2::RecomputePaths()
{

}
