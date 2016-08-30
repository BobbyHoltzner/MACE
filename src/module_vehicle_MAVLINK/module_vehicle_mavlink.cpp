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
//! \brief New commands have been updated that the vehicle is to follow immediatly
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::FollowNewCommands()
{

}


//!
//! \brief New commands have been issued to vehicle that are to be followed once current command is finished
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::FinishAndFollowNewCommands()
{

}


//!
//! \brief New commands have been appended to existing commands.
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::CommandsAppended()
{

}
