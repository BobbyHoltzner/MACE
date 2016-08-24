#include "module_vehicle_mavlink.h"


ModuleVehicleMAVLINK::ModuleVehicleMAVLINK(const MaceCore::MetadataVehicle &vehicleMetaData) :
    MaceCore::IModuleCommandVehicle(vehicleMetaData)
{
}


//!
//! \brief function that is to kick off the Vehicle Comms event loop
//!
void ModuleVehicleMAVLINK::start()
{
    while(true)
    {

        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}


//!
//! \brief Issue a new target position for the vehicle
//! \param position Position for the vehicle to achieve
//!
void ModuleVehicleMAVLINK::IssueTarget(const Eigen::Vector3d &position)
{
}
