#include "i_module_command_path_planning.h"
#include "i_module_command_RTA.h"
#include "i_module_command_vehicle.h"
#include "i_module_command_ground_station.h"
#include "i_module_command_external_link.h"
#include "i_module_command_sensors.h"

namespace MaceCore
{

ModuleBase::Classes IModuleCommandPathPlanning::moduleClass = ModuleBase::PATH_PLANNING;
ModuleBase::Classes IModuleCommandRTA::moduleClass = ModuleBase::RTA;
ModuleBase::Classes IModuleCommandVehicle::moduleClass = ModuleBase::VEHICLE_COMMS;
ModuleBase::Classes IModuleCommandGroundStation::moduleClass = ModuleBase::GROUND_STATION;
ModuleBase::Classes IModuleCommandExternalLink::moduleClass = ModuleBase::EXTERNAL_LINK;
ModuleBase::Classes IModuleCommandSensors::moduleClass = ModuleBase::SENSORS;

}
