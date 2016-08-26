#include "i_module_command_path_planning.h"
#include "i_module_command_RTA.h"
#include "i_module_command_vehicle.h"


namespace MaceCore
{

ModuleBase::Classes IModuleCommandPathPlanning::moduleClass = ModuleBase::PATH_PLANNING;
ModuleBase::Classes IModuleCommandRTA::moduleClass = ModuleBase::RTA;
ModuleBase::Classes IModuleCommandVehicle::moduleClass = ModuleBase::VEHICLE_COMMS;

//template<> ModuleBase::Classes IModuleCommandPathPlanning::AbstractModule_CRTP<IModuleCommandPathPlanning>::moduleClass = ModuleBase::PATH_PLANNING;
//template<> ModuleBase::Classes IModuleCommandRTA::AbstractModule_CRTP<IModuleCommandRTA>::moduleClass = ModuleBase::RTA;
//template<> ModuleBase::Classes IModuleCommandVehicle::AbstractModule_CRTP<IModuleCommandVehicle>::moduleClass = ModuleBase::VEHICLE_COMMS;

}
