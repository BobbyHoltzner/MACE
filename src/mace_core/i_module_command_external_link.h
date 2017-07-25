#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_base.h"
#include "abstract_module_event_listeners.h"
#include "abstract_module_base_vehicle_listener.h"

#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"
#include "i_module_events_external_link.h"

#include "command_marshler.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_AVAILABLE_ONBOARD_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION,
    NEWLY_AVAILABLE_VEHICLE
};

class MACE_CORESHARED_EXPORT IModuleCommandExternalLink : public AbstractModule_VehicleListener<Metadata_GroundStation, IModuleEventsExternalLink, ExternalLinkCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandExternalLink():
        AbstractModule_VehicleListener()
    {
        AddCommandLogic<Data::MissionKey>(ExternalLinkCommands::NEWLY_AVAILABLE_ONBOARD_MISSION, [this](const Data::MissionKey &key){
            NewlyAvailableOnboardMission(key);
        });

        AddCommandLogic<Data::MissionKey>(ExternalLinkCommands::NEW_MISSION_EXE_STATE, [this](const Data::MissionKey &key){
            NewlyAvailableMissionExeState(key);
        });

        AddCommandLogic<CommandItem::SpatialHome>(ExternalLinkCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const CommandItem::SpatialHome &home){
            NewlyAvailableHomePosition(home);
        });
        AddCommandLogic<int>(ExternalLinkCommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &systemID){
            NewlyAvailableVehicle(systemID);
        });



    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:

    virtual void NewlyAvailableOnboardMission(const Data::MissionKey &key) = 0;

    virtual void NewlyAvailableMissionExeState(const Data::MissionKey &missionKey) = 0;

    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home) = 0;

    virtual void NewlyAvailableVehicle(const int &systemID) = 0;

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
