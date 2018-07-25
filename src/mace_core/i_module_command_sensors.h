#ifndef I_MODULE_COMMAND_SENSORS_H
#define I_MODULE_COMMAND_SENSORS_H
#include "abstract_module_event_listeners.h"
#include "metadata_sensors.h"

#include "i_module_topic_events.h"
#include "i_module_events_sensors.h"
#include "i_module_events_vehicle.h"

#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class SensorCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS
};

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandSensors :
        public AbstractModule_EventListeners<Metadata_Sensors, IModuleEventsSensors, SensorCommands>,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
    public:

        static ModuleClasses moduleClass;

    IModuleCommandSensors():
        AbstractModule_EventListeners()
    {
        IModuleGenericVehicleListener::SetUp<Metadata_Sensors, IModuleEventsSensors, SensorCommands>(this);

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:


};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_SENSORS_H
