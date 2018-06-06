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
#include "module_characteristics.h"

#include "base/geometry/polygon_2DC.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_AVAILABLE_ONBOARD_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION,
    NEWLY_AVAILABLE_MODULE,
    NEW_OPERATIONAL_BOUNDARY,
    RECEIVED_MISSION_ACK
};

template <class U>
class Testing{

};

class MACE_CORESHARED_EXPORT IModuleCommandExternalLink : public AbstractModule_VehicleListener<Metadata_GroundStation, IModuleEventsExternalLink, ExternalLinkCommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandExternalLink():
        AbstractModule_VehicleListener()
    {

        AddCommandLogic<MissionItem::MissionKey>(ExternalLinkCommands::NEW_OPERATIONAL_BOUNDARY, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyRequiredOperationalBoundary(vehicleID, sender);
        });

        AddCommandLogic<MissionItem::MissionKey>(ExternalLinkCommands::NEWLY_AVAILABLE_ONBOARD_MISSION, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableOnboardMission(key, sender);
        });

        AddCommandLogic<MissionItem::MissionKey>(ExternalLinkCommands::NEW_MISSION_EXE_STATE, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){            
            UNUSED(sender);
            NewlyAvailableMissionExeState(key);
        });

        AddCommandLogic<CommandItem::SpatialHome>(ExternalLinkCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const CommandItem::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableHomePosition(home, sender);
        });

        AddCommandLogic<ModuleCharacteristic>(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, [this](const ModuleCharacteristic &module, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableModule(module);
        });

        AddCommandLogic<MissionItem::MissionACK>(ExternalLinkCommands::RECEIVED_MISSION_ACK, [this](const MissionItem::MissionACK &ack, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            ReceivedMissionACK(ack);
        });

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:

    virtual void NewOperationalBoundary(const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    virtual void NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) = 0;

    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    virtual void NewlyAvailableModule(const ModuleCharacteristic &module) = 0;

    virtual void ReceivedMissionACK(const MissionItem::MissionACK &ack) = 0;

};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
