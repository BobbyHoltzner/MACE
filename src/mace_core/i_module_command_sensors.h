#ifndef I_MODULE_COMMAND_SENSORS_H
#define I_MODULE_COMMAND_SENSORS_H
#include "abstract_module_event_listeners.h"
#include "metadata_sensors.h"

#include "i_module_topic_events.h"
#include "i_module_events_sensors.h"
#include "i_module_events_vehicle.h"

namespace MaceCore
{

enum class SensorCommands
{
    NEW_AVAILABLE_VEHICLE
};

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandSensors : public AbstractModule_EventListeners<Metadata_Sensors, IModuleEventsSensors, SensorCommands>
{
    friend class MaceCore;
    public:

        static ModuleClasses moduleClass;

    IModuleCommandSensors():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(SensorCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableVehicle(vehicleID);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;


};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_SENSORS_H
