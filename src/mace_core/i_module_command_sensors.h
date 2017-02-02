#ifndef I_MODULE_COMMAND_SENSORS_H
#define I_MODULE_COMMAND_SENSORS_H


#include "abstract_module_event_listeners.h"
#include "metadata_sensors.h"

#include "i_module_topic_events.h"


namespace MaceCore
{

enum class SensorCommands
{
    CREATE_VEHICLE_OBJECT
};

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandSensors : public AbstractModule_EventListeners<Metadata_Sensors, IModuleTopicEvents, SensorCommands>
{
    friend class MaceCore;

public:
    static Classes moduleClass;

    IModuleCommandSensors():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(SensorCommands::CREATE_VEHICLE_OBJECT, [this](const int &vehicleID){
            CreateVehicleObject(vehicleID);
        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void CreateVehicleObject(const int &vehicleID) = 0;

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_SENSORS_H
