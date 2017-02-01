#ifndef I_RTA_H
#define I_RTA_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_rta.h"
#include "i_module_topic_events.h"

namespace MaceCore
{

enum class RTACommands
{

};

class MACE_CORESHARED_EXPORT IModuleCommandRTA : public AbstractModule_EventListeners<Metadata_RTA, IModuleTopicEvents, RTACommands>
{
public:

    static Classes moduleClass;

    IModuleCommandRTA():
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

#endif // I_RTA_H

