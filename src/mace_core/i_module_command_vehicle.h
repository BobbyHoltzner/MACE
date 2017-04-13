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
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    REQUEST_DUMMY_FUNCTION
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
        this->template AddCommandLogic<int>(VehicleCommands::REQUEST_DUMMY_FUNCTION, [this](const int &vehicleID){
            RequestDummyFunction(vehicleID);
        });

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:
    virtual void RequestDummyFunction(const int &vehicleID) = 0;


};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
