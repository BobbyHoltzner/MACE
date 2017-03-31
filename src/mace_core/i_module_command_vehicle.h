#ifndef I_MODULE_COMMAND_VEHICLE_H
#define I_MODULE_COMMAND_VEHICLE_H

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"

namespace MaceCore
{

enum class VehicleCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS
};


class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandVehicle : public AbstractModule_VehicleListener<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>
{
friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandVehicle():
        AbstractModule_VehicleListener()
    {
        //These are from MACE Core to modules

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:


};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
