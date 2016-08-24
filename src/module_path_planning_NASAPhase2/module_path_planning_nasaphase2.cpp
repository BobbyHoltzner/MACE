#include "module_path_planning_nasaphase2.h"


ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2(const MaceCore::MetadataPathPlanning metaData) :
    MaceCore::IModuleCommandPathPlanning(metaData)
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
