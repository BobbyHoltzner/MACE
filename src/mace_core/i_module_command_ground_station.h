#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "data/mission_key.h"
#include "i_module_topic_events.h"
#include "i_module_events_ground_station.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    NEW_AVAILABLE_VEHICLE,
    NEWLY_AVAILABLE_CURRENT_MISSION,
    NEWLY_AVAILABLE_HOME_POSITION
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation : public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>
{
    friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(GroundStationCommands::NEW_AVAILABLE_VEHICLE, [this](const int &vehicleID){
            NewlyAvailableVehicle(vehicleID);
        });

        AddCommandLogic<Data::MissionKey>(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION, [this](const Data::MissionKey &missionKey){
            NewlyAvailableCurrentMission(missionKey);
        });

        AddCommandLogic<MissionItem::SpatialHome>(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const MissionItem::SpatialHome &home){
            NewlyAvailableHomePosition(home);
        });
    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    virtual void NewlyAvailableCurrentMission(const Data::MissionKey &missionKey) = 0;

    virtual void NewlyAvailableHomePosition(const MissionItem::SpatialHome &home) = 0;


    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
