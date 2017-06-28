#ifndef I_RTA_H
#define I_RTA_H

#include "abstract_module_event_listeners.h"
#include "metadata_rta.h"
#include "i_module_topic_events.h"
#include "i_module_events_rta.h"

namespace MaceCore
{

enum class RTACommands
{
    NEW_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandRTA : public AbstractModule_EventListeners<Metadata_RTA, IModuleEventsRTA, RTACommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandRTA():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(RTACommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID){
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

#endif // I_RTA_H

