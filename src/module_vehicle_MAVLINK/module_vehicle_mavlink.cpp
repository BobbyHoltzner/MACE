#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

ModuleVehicleMAVLINK::ModuleVehicleMAVLINK() :
    MaceCore::IModuleCommandVehicle()
{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleVehicleMAVLINK::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleMAVLINK::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
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
