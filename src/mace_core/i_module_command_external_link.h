#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "i_module_command_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_sensors.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
    NEW_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandExternalLink : public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsSensors, ExternalLinkCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandExternalLink():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(ExternalLinkCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID){
            NewlyAvailableVehicle(vehicleID);
        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
