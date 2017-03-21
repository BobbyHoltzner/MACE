#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "abstract_module_base_vehicle_listener.h"

#include "metadata_ground_station.h"

#include "i_module_command_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_sensors.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS
};

class MACE_CORESHARED_EXPORT IModuleCommandExternalLink : public AbstractModule_VehicleListener<Metadata_GroundStation, IModuleEventsVehicle, ExternalLinkCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandExternalLink():
        AbstractModule_VehicleListener()
    {

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
