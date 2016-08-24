#include "module_rta_nasaphase2.h"


ModuleRTANASAPhase2::ModuleRTANASAPhase2(const MaceCore::Metadata_RTA metaData) :
    MaceCore::IModuleCommandRTA(metaData)

{
}


//!
//! \brief function that is to kick off the RTA event loop of the module
//!
void ModuleRTANASAPhase2::start()
{
    while(true)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}


void ModuleRTANASAPhase2::NewVehicle(const std::string &ID, const MaceCore::MetadataVehicle &vehicle)
{

}


void ModuleRTANASAPhase2::RemoveVehicle(const std::string &ID)
{

}


void ModuleRTANASAPhase2::UpdatedPosition(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdateDynamicsState(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdatedVehicleLife(const std::string &vehicleID)
{

}


void ModuleRTANASAPhase2::UpdatedOccupancyMap()
{

}
