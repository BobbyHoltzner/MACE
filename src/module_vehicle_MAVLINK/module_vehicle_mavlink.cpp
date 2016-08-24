#include "module_vehicle_mavlink.h"


ModuleVehicleMAVLINK::ModuleVehicleMAVLINK(const MaceCore::MetadataVehicle &vehicleMetaData) :
    MaceCore::IModuleCommandVehicle(vehicleMetaData)
{
}


//!
//! \brief Issue a new target position for the vehicle
//! \param position Position for the vehicle to achieve
//!
void ModuleVehicleMAVLINK::IssueTarget(const Eigen::Vector3d &position)
{
}
