#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "i_module_topic_events.h"

#include "metadata_ground_station.h"

#include "vehicle_data.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
};


class MACE_CORESHARED_EXPORT IModuleCommandExternalLink : public AbstractModule_EventListeners<Metadata_GroundStation, IModuleTopicEvents, ExternalLinkCommands>
{
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
