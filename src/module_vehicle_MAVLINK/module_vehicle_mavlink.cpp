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
//! \brief New commands have been updated that the vehicle is to follow immediatly
//!
void ModuleVehicleMAVLINK::FollowNewCommands()
{

}


//!
//! \brief New commands have been issued to vehicle that are to be followed once current command is finished
//!
void ModuleVehicleMAVLINK::FinishAndFollowNewCommands()
{

}


//!
//! \brief New commands have been appended to existing commands
//!
void ModuleVehicleMAVLINK::CommandsAppended()
{

}
