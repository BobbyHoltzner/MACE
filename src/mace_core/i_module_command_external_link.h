#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

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

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
