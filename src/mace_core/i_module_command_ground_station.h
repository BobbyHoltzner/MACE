#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_ground_station.h"

#include "data_generic_command_item/command_item_components.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    NEW_AVAILABLE_VEHICLE,
    NEWLY_AVAILABLE_CURRENT_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation : public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(GroundStationCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableVehicle(vehicleID);
        });

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableCurrentMission(missionKey);
        });

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEW_MISSION_EXE_STATE, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableMissionExeState(missionKey);
        });

        AddCommandLogic<CommandItem::SpatialHome>(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const CommandItem::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableHomePosition(home);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    virtual void NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey) = 0;

    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) = 0;

    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home) = 0;

    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
