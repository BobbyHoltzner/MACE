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
    BASE_MODULE_LISTENER_ENUMS
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
        AddCommandLogic<int>(SensorCommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(SensorCommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    //!
    //! \brief NewlyAvailableVehicle New available vehicle subscriber
    //! \param vehicleID New vehicle ID
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;


    //!
    //! \brief NewlyUpdatedGlobalOrigin New global origin subscriber
    //! \param position New global origin position
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;


};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_SENSORS_H
